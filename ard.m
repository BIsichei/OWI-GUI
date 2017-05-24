function ard(Arduino)
%{ 

Author: Bendict Isichei

To use this GUI, the user passes an arduino board to the function when
calling it. 
The main interface shows a 3D representation of the robotic arm. 

The display tab allows the user to change the display settings of the
arm, and also to change the arm's current configuration.

The arduino tab is used to indicate what pins control what motor, and what 
analog inputs correspond to each motor. View Readme for info on pins.

The arduino tab has a calibration panel that shows up only after the pins
have been set.

The automation tab allows the end User to program simple robot motions.

(NB. The pins were programmed with a MEGA board in mind, and
thus will only recognise digital pins from 2 to 53, and analog pins
from 0 to 15)

Due to the nature of DH parameters and how they build up from the base,
an index value of 1 typically corresponds to a parameter related to M5,
while an index of 5 corresponts to one of M1.
%}

clc
% Makes sure there's only one instance of the GUI up at a time
f = findall(0,'Name','OWI_GUI');
if isempty(f)
    f = figure('Visible','off','Units','Pixels','ToolBar','none',...
        'Name','OWI_GUI','MenuBar','none','Numbertitle','off');
else
    clf(f);
    f.Visible = 'off'; f.Units = 'Pixels';
    f.ToolBar = 'none';f.MenuBar = 'none';
    f.NumberTitle = 'off';
end

% Setup the axes and the main tab groups
Axes1 = axes(f,'Units','Pixels','Position',[50,60,400,400],'Box','on');
TabGroup = uitabgroup(f,'Units','Pixels','position',[470 0, 280, 500],...
    'SelectionChangedFcn',@TabChanged_Callback);
ViewTab = uitab(TabGroup,'Title','Display');
ArduinoTab = uitab(TabGroup,'Title','Arduino'); 
ProgramTab = uitab(TabGroup,'Title','Automation');
% InverseTab = uitab(TabGroup, 'Title','Inverse Kinematics');
% CamTab = uitab(TabGroup,'Title','ImageProcessing');

%% DATA MEMBERS
axis square
view(-150,30);
rotate3d on;
ArduinoDetected = nargin;
tolerance = 0.075; %volts
Data = struct;
Data.input = [0, -pi/2, 0, Inf;
    90, 0, 0, Inf;
    118, 0, 0, Inf;
    41, 0, 0, Inf];
Data.Lims  = [-3*pi/4, 3*pi/4;
    0, pi;
    -5*pi/6, 5*pi/6;
    -pi/3, pi/3;
    0, 45];
Data.DefaultPins = [     0     4     3;
     1     5     6;
     2     7     8;
     3     9    10;
     4    12    11;
     0     1     0];
Data.Set = 0;
File = []; % temp data storage for saving and loading
Settings = struct;
Program = struct;
Program.Config = {};
saveindex = 1;
Data.Stop = uicontrol('Position',[662.5,125,70,80],'String','STOP',...
    'BackgroundColor',[.71 .4 .4],'Callback',@Stop);
Data.RevUpdate = uicontrol('String','UpdatePlot',...
    'Callback',@PlotUpdate_Callback);

%% VIEW TAB SETUP
ViewSettings = uipanel(ViewTab,'Units','Pixels','Position',[10,230,155,105],...
    'Title', 'VIEWS','TitlePosition','centertop');
AxisSettings = uipanel(ViewTab,'Units','Pixels','Position',[10,10,180,115],...
    'Title','AXIS LIMITS','TitlePosition','centertop','Visible','off');
SlideSettings = uipanel(ViewTab,'Units','Pixels','Position',[10,10,180,105],...
    'Title','SLIDE LIMITS','TitlePosition','centertop','Visible','off');
RemoteSettings = uipanel(ViewTab,'Units','Pixels','Position',[2 345 26 130]);


%VIEWTAB CHILDREN
b = 350; %base y value for sliders
for j = 1:5
    Data.SlideLabel(j) = uicontrol(ViewTab,'String',strcat('M',num2str(6-j)),...
        'Position',[32,b+30*(j-1)+10,20,12],...
        'TooltipString','Reset this joint','Callback',@ResetArms_Callback,...
        'UserData',j,'BackgroundColor',[.90 .90 .90]);
    Data.SlideText(j) = uicontrol(ViewTab,'Style','edit',...
        'Position',[32,b+30*(j-1)-2,20,14],...
        'Callback',@SlideText_Callback,'UserData',j,'String',0);    
    align([Data.SlideText(j) Data.SlideLabel(j)],'left','Fixed',1);
    Data.Slide(j) =  uicontrol(ViewTab,'Style','slider',...
        'Position',[55,b+30*(j-1),200,10],...
        'Callback',@Slide_Callback,'Min', Data.Lims(j,1),'Max',Data.Lims(j,2),...
        'String', 'R','Value',0,'UserData',j);
    Data.Upload(j) = uicontrol(ViewTab,...
        'position',[258,b+30*(j-1)-2,14,14],'String','>',...
        'Callback',@Upload_Callback,...
        'FontWeight','bold','FontSize',12,'UserData',j,...
        'TooltipString','Upload to board');
    Data.HGTrans(j) = hgtransform(Axes1);
    Data.PSet(j,:) = {'0' ,'0'};
    Data.PLims(j,:) = [Inf Inf];
end
for j = 6:10
    Data.HGTrans(j) = hgtransform(Axes1);
end
Data.Slide(5).SliderStep = [1/9 1/9];
Data.Slide(5).String = 'Grip';
Data.UploadAll = uicontrol(ViewTab,...
    'Position',[175,290,90,39],'UserData',6,'String','Upload all',...
    'Callback',@Upload_Callback,'TooltipString','Upload all Values');
Data.ResetArm = uicontrol(ViewTab,...
    'Position',[175,260,90,20],'String','Reset Arm',...
    'Callback',@ResetArms_Callback,'UserData',6);
Data.Radians = uicontrol(ViewTab,'Style', 'toggle','Position', [175,230,90,20],...
    'String','Radians','Callback',@Radian_Callback);
Settings.StatusDisplay = uicontrol(ViewTab,'Style','text',...
    'Position',[10,125,180,95],'Max',2,'HorizontalAlignment','left',...
    'BackgroundColor',[.91 .91 .93]);
Settings.Axis.Reset = uicontrol(ViewTab,'Position',[195 180 70 40],...
    'String','Reset Axis','Callback',@ResetAxis_Callback,...
    'Visible','off');
Settings.Slide.Reset = uicontrol(ViewTab,'Position',[195 180 70 40],...
    'String','Reset all Slides',...
    'Callback',@ResetLimits_Callback,'Visible','off');
Settings.Advanced1 = uicontrol(ViewTab,'Style','radio',...
    'TooltipString','Axis Limits','Position',[2 5 20 20],...
    'UserData','Axis','Callback',@AdvancedControls_Callback);
Settings.Advanced2 = uicontrol(ViewTab,'Style','radio',...
    'TooltipString','Slide Limits','Position',[25 5 20 20],...
    'UserData','Slide','Callback',@AdvancedControls_Callback);


%VIEWSETTINGS CHILDREN
Settings.View.YZ = uicontrol(ViewSettings,'Position',[10,10,30,20],...
    'String','YZ','Callback',@View_Callback);
Settings.View.YZ_ = uicontrol(ViewSettings,'Position',[50,10,30,20],...
    'String','YZ''','Callback',@View_Callback);
Settings.View.XZ = uicontrol(ViewSettings,'Position',[10,40,30,20],...
    'String','XZ','Callback',@View_Callback);
Settings.View.XZ_ = uicontrol(ViewSettings,'Position',[50,40,30,20],...
    'String','XZ''','Callback',@View_Callback);
Settings.View.YX = uicontrol(ViewSettings,'Position',[10,70,30,20],...
    'String','YX','Callback',@View_Callback);
Settings.View.Reset = uicontrol(ViewSettings','Position',[50,70,90,20],...
    'String','RESET','Callback',@View_Callback);
Settings.View.Rotate = uicontrol(ViewSettings','Style','toggle','Position',[90,40,50,20],...
    'String','ROTATE','Callback',@View_Callback,'Value',1);
Settings.View.Iso = uicontrol(ViewSettings,'Position',[90,10,50,20],...
    'String','ISO','Callback',@View_Callback);


%AXISSETTINGS CHILDREN
Settings.Axis.TopLabel = uicontrol(AxisSettings,'Style','text',...
    'String','Min : Max','Position',[10,83,40,20]);
Values = {...
    'Min',  'edit', [50,20],    Inf,    Inf;
    'Max',  'edit', [50,20],    Inf,    Inf;
    'Label','push', [50,20],    'Set ', 'Limit_Callback'};
ButtonSet('x',[10 10 5],'Settings',Values,'Axis','on','off');
ButtonSet('y',[10 40 5],'Settings',Values,'Axis','on','off');
ButtonSet('z',[10 70 5],'Settings',Values,'Axis','on','off');


%SLIDESETTINGS CHILDREN
Settings.Slide.Menu = uicontrol(SlideSettings,'Style','pop',...
    'Position',[10,60,50,20],'String',{'M5';'M4';'M3';'M2';'M1'});
Settings.Slide.ShowBoxes = uicontrol(SlideSettings,'Style','toggle',...
    'Position',[70,56,100,25],'String','View Slide Limit Boxes',...
    'Callback',@ViewSlider_Callback);
Settings.Slide.TopLabel = uicontrol(SlideSettings,'Style','text',...
    'String','Min : Max','Position',[10,23,40,20],'Visible','off');
Settings.Slide.ResetText = uicontrol(SlideSettings,'Style','text',...
    'Position',[70,35,35,15],'String','Reset');
Settings.Slide.Reset2 = uicontrol(SlideSettings,...
    'Position',[70,35,100,15],'String','Reset this Slide',...
    'Callback',@ResetLimits_Callback);
Values = {...
    'Min',  'edit', [50,20],    Inf,    Inf;
    'Max',  'edit', [50,20],    Inf,    Inf;
    'Label','push', [50,20],    'Limit Slider ', 'Limit_Callback'};
for j = 1:5
    ButtonSet(num2str(j),[10 10 5],'Settings',Values,'Slide','off','on');
end


%REMOTESETTINGS CHILDREN
Values = {'Origin',  'push', [20,20],    Inf,    Inf};
for j = 1:5
    ButtonSet(num2str(j),[2 3+25*(j-1) 5],'Settings',Values,'Remote','on','on');
    Settings.Remote.Origin(j).KeyPressFcn = @RemoteKeyPress_Callback;
    Settings.Remote.Origin(j).Callback = @Remote_Callback;
    Settings.Remote.Origin(j).String = strcat('M',num2str(6-j));
end

%% ARDUINO TAB SETUP
PinSettings = uipanel(ArduinoTab,'Units','Pixels','Position',[5,290,270,210],...
    'Title','Pin Settings');
CalPanel = uipanel(ArduinoTab,'Units','Pixels','Position',[5 45 270 235],...
    'Title','Calibration','Visible','off');


%PINSETTINGS CHILDREN
OutBitSetting = uibuttongroup(PinSettings,'Units','Pixels',...
    'Position',[5 160 190 30],'Title','What value activates the motors?');
Settings.Pin.Low = uicontrol(OutBitSetting,'Style','radio',...
    'Position',[65 3 50 14],'String','LOW');
Settings.Pin.High = uicontrol(OutBitSetting,'Style','radio',...
    'Position',[5 3 50 14],'String','HIGH');
Values = {...
    'Neg', 'text', [21,20], '-', Inf;
    'Pos', 'text', [21,20], '+', Inf};
ButtonSet('Pin',[155 140 0],'Settings',Values,'Pin','on','off');
Settings.Pin.PosPin.TooltipString = 'These values indicate what pins make the part move in the Positive theta direction according to the sliders';
Settings.Pin.NegPin.TooltipString = 'These values indicate what pins make the part move in the Negative theta direction according to the sliders';
Values = {...
    'Neg',  'edit', [20,20],    Inf,    Inf;
    'Pos',  'edit', [20,20],    Inf,    Inf;
    'Label','text', [70,20],    Inf,    Inf};
Values1 = {...
    'ALabel', 'text', [80,20], Inf, Inf;
    'APin',   'edit', [20,20], Inf, Inf};
for j = 1:5
    ButtonSet(num2str(j),[5 25+25*(j-1) 1],'Settings',Values1,'Pin','on','on');
    ButtonSet(num2str(j),[155 25+25*(j-1) 1],'Settings',Values,'Pin','on','on');
    Settings.Pin.ALabel(j).String = strcat('M',num2str(6-j),' Analog Pin: A');
    Settings.Pin.ALabel(j).HorizontalAlignment = 'right';
    Settings.Pin.Label(j).String = strcat('M',num2str(6-j),' Output Pins');
    Settings.Pin.Label(j).HorizontalAlignment = 'left';
end
Settings.Pin.Set = uicontrol(PinSettings,...
    'Position',[5,3,260,20],'String','Apply All Values',...
    'Callback', @SetPin_Callback);
Settings.Pin.Save = uicontrol(PinSettings,'Position',[200 170 50 20],...
    'Callback',@Save_Callback,'String','Save Pins');
Settings.Pin.Load = uicontrol(PinSettings,'Position',[200 145 50 20],...
    'Callback',@Load_Callback,'String','Load Pins');


%CALPANEL CHILDREN
CalSettings = uibuttongroup(CalPanel,'Units','Pixels','Position',[0 188 270 50]);
Values = {'Button',  'toggle', [50,20],    'Slide ',    'Button_Callback'};
for j = 1:5
    ButtonSet(num2str(j),[1+53*(j-1) 5],'Settings',Values,'Cal','on','on');
    Settings.Cal.Button(j).String = strcat('M',num2str(6-j));
end
Settings.Cal.Orient = uicontrol(CalPanel,'Units','Pixels','String','Orient',...
    'Position',[5 30 30 20],'Callback',@Orientation_Callback);
Settings.Cal.Set = uicontrol (CalPanel,'Units','Pixels',...
    'Position',[40 30 30 20],'String','Set','Callback',@Calibrate_Callback);
Settings.Cal.ReadingLabel = uicontrol(CalPanel,'Style','text','Units','Pixels',...
    'Position',[80 30 45 20],'String','Reading:');
Settings.Cal.Reading = uicontrol(CalPanel,'Style','text','Units','Pixels',...
    'Position',[125 30 50 20],'String','--','BackgroundColor',[.98 .98 .98]);
Settings.Cal.Next = uicontrol(CalPanel,'Units','Pixels','Position',[5 5 180 20],...
    'String','Next >>','Callback',@NextButton_Callback);
Settings.Cal.Save = uicontrol(CalPanel,'Position',[190 30 70 20],...
    'Callback',@Save_Callback,'String','Save Calibration');
Settings.Cal.Load = uicontrol(CalPanel,'Position',[190 5 70 20],...
    'Callback',@Load_Callback,'String','Load Calibration');

%% PROGRAM TAB SETUP
ClosedLoopPanel = uipanel(ProgramTab,'Title','Closed Loop Automation');
OpenLoopPanel = uipanel(ProgramTab,'Visible','off','Title','Open Loop Automation');
% Program.Switch = uicontrol(ProgramTab,'Position',[100 5 70 25],...
%     'Callback',@ProgramSwitch_Callback,'String','Switch Mode','UserData',0);


%CLOSED LOOP CHILDREN
Program.Append = uicontrol(ClosedLoopPanel,'Position',[54 422 70 60],...
    'String','Save Config','Callback',@ProgramSave_Callback,'Tag','Append');
Program.Display = uicontrol(ClosedLoopPanel,'Position',[54 334 70 60],...
    'String','Show Motion','Callback',@ProgramMove_Callback,'Tag','Display');
Program.Delete = uicontrol(ClosedLoopPanel,'Position',[54 250 70 60],...
    'String','Delete last Config','Callback',@ProgramSave_Callback,'Tag','Delete');
Program.List = uicontrol(ClosedLoopPanel,'Style','list','Position',[134 250 50 230],...
    'Callback',@ProgramList_Callback,'Tag','List','Max',2);
Program.Save = uicontrol(ClosedLoopPanel,'Position',[195 422 70 60],...
    'Callback',@Save_Callback,'String','Save Motion');
Program.Load = uicontrol(ClosedLoopPanel,'Position',[195 334 70 60],...
    'Callback',@Load_Callback,'String','Load Motion');
Program.Reset = uicontrol(ClosedLoopPanel,'Position',[195 250 70 60],...
    'String','Reset Motion','Callback',@ProgramSave_Callback,'Tag','Reset');
Program.Upload = uicontrol(ClosedLoopPanel,'Position',[10 225 255 20],...
    'String','Upload Motion','Callback',@ProgramMove_Callback,'Tag','Upload');


%OPEN LOOP CHILDREN

%% DRAWING ITEMS
y = [231 188 34]./255;
%Base1
[Vertices, Faces , ~, ~]= stlReadBinary('Base_1.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'FaceColor',y,'Parent',Data.HGTrans(1),'EdgeColor',y/2);
%Base2
[Vertices, Faces , ~, ~]= stlReadBinary('Base_2.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'FaceColor',[.1 .1 .1],'Parent',Data.HGTrans(1));
% Arm1
[Vertices, Faces , ~, ~]= stlReadBinary('arm1.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.HGTrans(2),'FaceColor',[.1 .1 .1]);
%Arm2_1
[Vertices, Faces, ~, ~]= stlReadBinary('arm2_1.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'FaceColor',[.1 .1 .1],'Parent',Data.HGTrans(3));
%Arm2_2
[Vertices, Faces, ~, ~]= stlReadBinary('arm2_2.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'FaceColor',y,'Parent',Data.HGTrans(3),'EdgeColor',y/2);
%Gripper
[Vertices, Faces, ~, ~]= stlReadBinary('gripper.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.HGTrans(4),'FaceColor',[.1 .1 .1]);
%Gripper_1
[Vertices, Faces, ~, ~]= stlReadBinary('gripper_1.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.HGTrans(5),'FaceColor',[.19 .19 .19]);
%Gripper_2
[Vertices, Faces, ~, ~]= stlReadBinary('gripper_2.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.HGTrans(6),'FaceColor',[.19 .19 .19]);
%Gripper_3 and 4
[Vertices, Faces, ~, ~]= stlReadBinary('gripper_3.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.HGTrans(7),'FaceColor',[.19 .19 .19]);
[Vertices, Faces, ~, ~]= stlReadBinary('gripper_3.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.HGTrans(8),'FaceColor',[.19 .19 .19]);
%Gripper_5 and 6
[Vertices, Faces, ~, ~]= stlReadBinary('gripper_4l.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.HGTrans(9),'FaceColor',[.19 .19 .19]);
[Vertices, Faces, ~, ~]= stlReadBinary('gripper_4r.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.HGTrans(10),'FaceColor',[.19 .19 .19]);
%Battery
[Vertices, Faces, ~, ~]= stlReadBinary('battery.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'FaceColor',[.1 .1 .1]);
%Table
TableTrans = hgtransform('Parent',Axes1,'Matrix',makehgtform('translate',[0 0 -70]));
rectangle('Position',[-300 -300 600 600],'Parent',TableTrans,...
    'Curvature',[1,1],'FaceColor',[.8 .8 .8]);
f.Position = [100 100 750 500];

%% NORMALIZING ALL UI ELEMENTS and initial run
f.Units = 'normalized';
Axes1.Units = 'normalized';
TabGroup.Units = 'normalized';
g = findobj('Type','uicontrol','-or','Type','uibuttongroup','-or','Type','uipanel');
for j = 1:size(g,1)
    g(j,1).Units = 'normalized';
end
g = findobj('Type','uicontrol');
for j = 1:size(g,1)
    g(j,1).FontUnits = 'normalized';
end
f.Position = [0.15 0.15 .7 .7];
RemoteSettings.Position = [2/280 345/500 26/280 152/500];
Settings.Advanced1.FontUnits = 'points';
Settings.Advanced2.FontUnits = 'points';
ResetAxis_Callback
ResetLimits_Callback(Settings.Slide.Reset);
Update
f.Visible = 'on'; %show the figure

for j = 1:5
    Settings.Pin.APin(j).String = Data.DefaultPins(j,1);
    Settings.Pin.Neg(j).String = Data.DefaultPins(j,2);
    Settings.Pin.Pos(j).String = Data.DefaultPins(j,3);
    Data.SlideText(j).FontUnits = 'points';
end

%% CALLBACKS

%%Shows or hides advanced controls not meant for the general user
    function AdvancedControls_Callback(source,~)        
        %Called by Settings.Advanced1 or Advanced2 in the ViewTab
        if strcmp(source.Units,'normalized')
            xmax = 280;
            ymax = 500;
        else
            xmax = 1;
            ymax = 1;
        end
        if source.Value
            switch source.UserData
                case 'Axis'
                    AxisSettings.Visible = 'on';
                    Settings.Axis.Reset.Visible = 'on';
                    align([Data.Radians source Settings.Axis.Reset],'Right','None')
                    Settings.Advanced2.Visible = 'off';
                case 'Slide'
                    SlideSettings.Visible = 'on';
                    Settings.Slide.Reset.Visible = 'on';
                    Settings.Advanced1.Visible = 'off';
                    align([Data.Radians source Settings.Slide.Reset],'Right','None')
            end
            source.Position = [195/xmax 97/ymax 70/xmax 80/ymax];%[195 97 80 80];
            align([Data.Radians source],'Right','None')
            Data.Stop.Position = [662.5/750,12/500,70/750,80/500];
            source.Style = 'toggle';
            source.String = 'Close Limits';
        else
            AxisSettings.Visible = 'off';
            SlideSettings.Visible = 'off';
            Settings.Axis.Reset.Visible = 'off';
            Settings.Slide.Reset.Visible = 'off';
            Settings.Advanced1.Visible = 'on';
            Settings.Advanced2.Visible = 'on';
            if strcmp(source.UserData,'Axis')
                source.Position = [ 5/xmax 5/ymax 20/xmax 20/ymax];
            else
                source.Position = [25/xmax 5/ymax 20/xmax 20/ymax];
            end
            source.Style = 'radio';
            Data.Stop.Position = [662.5/750,125/500,70/750,80/500];
            source.String ='';
        end
    end

%%sets the x, y or z limits.
    function AxisLimit_Callback(source,~)
        %Called by Settings.Axis.Label<> in the AxisLimit uipanel
        val = source.UserData;
        %val can be 'x' or 'y' or 'z'
        %valLim([str2double(Settings.Axis.Minval.String) str2double(Settings.Axis.Maxval.String) ])
        front = 'str2double(Settings.Axis.';
        back = '.String);';
        min = strcat(front,'Min',val,back,' ');%str2double(Settings.Axis.Min<val>.String);
        max = strcat(front,'Max',val,back);    %str2double(Settings.Axis.Max<val>.String);
        eval(strcat(val,'lim([',min,max,']);'))%<val>lim([min max]);
    end

%%Opens the Calibration Menu for the selected Motor
    function CalButton_Callback(source,~)        
        %Called by Settings.Cal.Button in the CalSettings uipanel
        val = str2double(source.UserData);
        if strcmp(TabGroup.SelectedTab.Title,'Arduino')&& Data.Set
            if ~strcmp(Data.PSet(val,1),'Set')||~strcmp(Data.PSet(val,2),'Set')
                Data.Pset(val,1) = {''};
                Data.Pset(val,2) = {''};
                Settings.StatusDisplay.String = strcat('Click Orient to Start Calibrating M',num2str(6-val));
                Settings.Cal.Reading.String = '--';
                Settings.Cal.Set.Visible = 'off';
                Settings.Cal.Next.Visible = 'off';
                Settings.Cal.Orient.Visible = 'on';
            else
                iscalibrated = strcat('M',num2str(6-val),' has been calibrated already. click Reset to recalibrate, or select a different Motor to calibrate');
                Settings.StatusDisplay.String = iscalibrated;
                Settings.Cal.Reading.String = Data.PLims(val,2);
                Settings.Cal.Next.Visible = 'on';
                Settings.Cal.Orient.Visible = 'off';
            end
        end
    end

%%Sets the Min or Max value for the selected motor
    function Calibrate_Callback(source,~)        
        %Called by Settings.Cal.Set on the CalSettings uipanel
        val = str2double(CalSettings.SelectedObject.UserData);
        Motor = strcat('M',num2str(6-val));
        if ~strcmp(Data.PSet(val,1),'Set')
            Data.PLims(val,1) = analogRead(Data.Pins(val,1));
            Data.PSet(val,1) = {'Set'};
            Settings.Cal.Reading.String = Data.PLims(val,1);
            Settings.StatusDisplay.String = strcat(Motor,' Min has been Set, Click Next to proceed');
            Settings.Cal.Next.String = 'Next >>';
            Settings.Cal.Next.Visible = 'on';
            Settings.Cal.Orient.Visible = 'off';
            source.Visible = 'off';
        else
            Data.PLims(val,2) = analogRead(Data.Pins(val,1));
            Data.PSet(val,2) = {'Set'};
            Settings.Cal.Reading.String = Data.PLims(val,2);
            Settings.StatusDisplay.String = strcat(Motor,' Max been calibrated. Click Reset to recalibrate ',Motor,' Min, or select a different Motor to calibrate');
            Settings.Cal.Next.String = 'Reset';
            Settings.Cal.Next.Visible = 'on';
            Settings.Cal.Orient.Visible = 'off';
            source.Visible = 'off';
        end
    end

%%Loads File from disk
    function Load_Callback(source,~)
        switch source.Parent.Title
            case 'Pin Settings'
                uiopen('Pins.mat');
                if numel(File) == 18
                    for i = 1:5
                        Settings.Pin.APin(i).String = File(i,1);
                        Settings.Pin.Neg(i).String = File(i,2);
                        Settings.Pin.Pos(i).String = File(i,3);
                    end
                    File = [];
                    Settings.StatusDisplay.String = 'Pin info loaded';
                else
                    Settings.StatusDisplay.String = 'Invalid file';
                end
            case 'Calibration'
                uiopen('Calibration.mat');
                if numel(File) == 2
                    Data.PLims(:,:) = File{1,1};
                    Data.PSet(:,:) = File{2,1};
                    File = [];
                    Settings.StatusDisplay.String = 'Calibration file loaded';
                else
                    Settings.StatusDisplay.String = 'Invalid file';
                end
            case 'Open Loop Automation'
            case 'Closed Loop Automation'
                uiopen('Movement.mat');
                if size(File,1) == 1
                    Program.Config = {Program.Config{:,:}, File{:,:}};
                    File = [];
                    for i = 1:size(Program.Config,2)
                        Program.List.String{i} = strcat('Config ',num2str(i));
                    end
                    saveindex = size(Program.Config,2)+1;
                else
                    Settings.StatusDisplay.String = 'Invalid file';
                end
        end
    end

%%Swtiches to calibrate the next value, or reset the previous value
    function NextButton_Callback(source,~)        
        %Called by Settings.Cal.Next on the CalSettings uipanel
        val = str2double(CalSettings.SelectedObject.UserData);
        if strcmp(Data.PSet(val,1),'Set')&&~strcmp(Data.PSet(val,2),'Set')
            Settings.StatusDisplay.String = 'Click Orient';
            Settings.Cal.Reading.String = '--';
            source.Visible = 'off';
            Settings.Cal.Orient.Visible = 'on';
        else
            Data.PSet(val,1) = {'0'};
            Data.PSet(val,2) = {'0'};
            Settings.StatusDisplay.String = 'Values reset, Click Orient';
            Settings.Cal.Reading.String = '--';
            source.Visible = 'off';
            Settings.Cal.Orient.Visible = 'on';
        end
    end

%%Shows the end user the proper orientation required to calibrate
%the robot's physical readings
    function Orientation_Callback(~,~)        
        %Called by Settings.Cal.Orient on the CalSettings uipanel
        val = str2double(CalSettings.SelectedObject.UserData);
        if Data.Set
            switch val
                case 1
                    if ~strcmp(Data.PSet(val,1),'Set')
                        Config([-135 0 0 0 0])
                        Settings.StatusDisplay.String = 'Now move the robot to the configuration shown on the left, then click Set';
                    else
                        Config([135 0 0 0 0]);
                        Settings.StatusDisplay.String = 'Now move the robot to the configuration shown on the left, then click Set';
                    end
                    view(-90,90)
                    Settings.Cal.Set.Visible = 'on';
                case 2
                    if ~strcmp(Data.PSet(val,1),'Set')
                        Config([0 0 90 0 0]);
                        Settings.StatusDisplay.String = 'For this calibration, try to get the arm vertical with the base of the housing for M3 touching the top of the battery pack, then click Set';
                    else
                        Config([0 180 -90 0 0]);
                        Settings.StatusDisplay.String = 'For this Calibration, move M4 down until the joint Clicks, then click Set';
                    end
                    view(180,0)
                    Settings.Cal.Set.Visible = 'on';
                case 3
                    if ~strcmp(Data.PSet(val,1),'Set')
                        Config([0 180 -150 0 0]);
                        Settings.StatusDisplay.String = 'For this calibration, turn M3 until it''s hitting the housing for M4, then click Set';
                    else
                        Config([0 0 150 0 0]);
                        Settings.StatusDisplay.String = 'For this Calibration, turn M3 until it''s hitting the housing for M4, then click Set';
                    end
                    view(180,0)
                    Settings.Cal.Set.Visible = 'on';
                case 4
                    if ~strcmp(Data.PSet(val,1),'Set')
                        Config([0 90 0 -60 0]);
                        Settings.StatusDisplay.String = 'For this calibration, turn M2 until it clicks';
                    else
                        Config([0 90 0 60 0]);
                        Settings.StatusDisplay.String = 'For this Calibration, turn M2 until it clicks';
                    end
                    view(180,0)
                    Settings.Cal.Set.Visible = 'on';
                case 5
                    if ~strcmp(Data.PSet(val,1),'Set')
                        Config([0 90 0 0 0]);
                        Settings.StatusDisplay.String = 'For this calibration, turn M1 until it clicks';
                    else
                        Config([0 90 0 0 45]);
                        Settings.StatusDisplay.String = 'For this Calibration, turn M1 until it clicks';
                    end
                    view(-90,0)
                    Settings.Cal.Set.Visible = 'on';
            end
        else
            Status('Please Setup Pins');
        end
        
    end

%%Updates the plot with the physical robot's current orientation
    function PlotUpdate_Callback(~,~)
        %called by Data.UpdatePlot in the Main Figure
        if  PinCheck 
            Config(GetConfig);
        else
            Status('Can''t');
        end
    end

%%Callback for displaying a motion Config
    function ProgramList_Callback(source,~)
        Config(Program.Config{source.Value});
    end

%%Callback for buttons in the program tab pertaining to moving the arm
    function ProgramMove_Callback(source,~)
        %Called by Program.Display and Program.Upload in the RemoteTab
        if numel(Program.Config)
            if strcmp(source.Tag,'Display') 
                for i = 1:saveindex-1
                    Config(Program.Config{i},'slowly')
                end
            elseif strcmp(source.Tag,'Upload')
                if  PinCheck
                    Settings.StatusDisplay.String = 'Uploading';
                    for i = 1:saveindex-1
                        Config(Program.Config{i});
                        Upload_Callback(Data.UploadAll);
                    end
                    Settings.StatusDisplay.String = 'Done';
                else
                    Status('It is required');
                end
            end
        else
            Settings.StatusDisplay.String = 'No saved motion';
        end
    end

%%Callback for buttons in the program tab pertaining to motion data
    function ProgramSave_Callback(source,~)
        %Called by Program.Delete, Program.Append and Program.Reset
        if strcmp(source.Tag,'Append')
            Program.Config{saveindex} = GetConfig(1);
            Settings.StatusDisplay.String = Program.Config{saveindex};
            Status(strcat('Config ',num2str(saveindex),' saved'));
            Program.List.String{saveindex} = strcat('Config ',num2str(saveindex));
            saveindex = saveindex + 1;
        elseif strcmp(source.Tag,'Delete')&& saveindex>2
            saveindex = saveindex -1;
            Config(Program.Config{saveindex-1});
            Program.List.String{saveindex} = [];
            Settings.StatusDisplay.String = strcat('Config ',num2str(saveindex),' deleted');
        else
            saveindex = 1;
            Program.Config = {};
            Program.List.String = {};
            Program.List.Value = saveindex;
            Config([0 0 0 0 0]);
            Settings.StatusDisplay.String = 'Motion cancelled';
        end
    end

%%Switches between open loop Programming and closed loop Programming
%     function ProgramSwitch_Callback(source,~)
%         if source.UserData
%             state = {'off','on'};
%         else 
%             state = {'on', 'off'};
%         end
%         OpenLoopPanel.Visible = state{1};
%         ClosedLoopPanel.Visible = state{2};
%         source.UserData = ~source.UserData;
%     end

%%Changes the Angles shown next to the sliders to degrees or radians
    function Radian_Callback(source,~)
        %Called by Data.Radians in the ViewTab
        for i = 1:4
            Data.SlideText(i).String = Data.Slide(i).Value*Check(i);
        end
        if source.Value
            source.String = 'Degrees';
        else
            source.String = 'Radians';
        end
    end

%Informs the user what buttons move the selected motor
    function Remote_Callback(source,~)
        %called by Settings.Remote.Origin<> in the RemoteSettings uipanel
        if Data.Set
            switch source.String
                case 'M5'
                    Settings.StatusDisplay.String = 'Use the left and right arrow keys to move. Push any other key to stop';
                otherwise
                    Settings.StatusDisplay.String = 'Use the up and down arrow keys to move. Push any other key to stop';
            end
        else
            Settings.StatusDisplay.String = 'Set Arduino pins first';
        end
    end

%Moves the robot arm using keyboard input based on the motor selected
    function RemoteKeyPress_Callback(~,event)
        %Called by Settings.Remote.Origin<> in the RemoteSettings uipanel
        if Data.Set
            if strcmp(event.Source.String,'M5')
                options = {'rightarrow','leftarrow'};
            elseif strcmp(event.Source.String,'M1')
                options = {'downarrow','uparrow'};
            else
                options = {'uparrow','downarrow'};
            end
            key = event.Key;
            val = str2double(event.Source.UserData);
            
            if strcmp(key,options{1,1})
                digitalWrite(Data.Pins(val,2),~Data.Pins(6,1));
                digitalWrite(Data.Pins(val,3),Data.Pins(6,1));
            elseif strcmp(key,options{1,2})
                digitalWrite(Data.Pins(val,3),~Data.Pins(6,1));
                digitalWrite(Data.Pins(val,2),Data.Pins(6,1));
            else
                digitalWrite(Data.Pins(val,3),~Data.Pins(6,1));
                digitalWrite(Data.Pins(val,2),~Data.Pins(6,1));
                PlotUpdate_Callback();
            end
        else
            Settings.StatusDisplay.String = 'Please setup pins first';
        end
    end

%%Resets all the robot's arms
    function ResetArms_Callback(source,~)        
        %Called by Data.ResetArm or Data.SlideLable<> in the ViewTab
        val = source.UserData;
        if val < 6
            Data.Slide(val).Value = 0;
            Data.SlideText(val).String = 0;
        else
            for i=1:5
                ResetArms_Callback(Data.SlideLabel(i));
            end
        end
        Update
    end

%%resets the axis limits
    function ResetAxis_Callback(~,~)        
        %Called by Settings.Axis.Reset in the ViewTab
        axis([-350 350 -350 350  -75 625]);
        %         axis([200 350 -75 75  -75 75]);
        Settings.Axis.Minx.String = -350;
        Settings.Axis.Maxx.String = 350;
        Settings.Axis.Miny.String = -350;
        Settings.Axis.Maxy.String = 350;
        Settings.Axis.Minz.String = -75;
        Settings.Axis.Maxz.String = 625;
    end

%%Resets the current slide limits or all slide limits
    function ResetLimits_Callback(source,~)        
        %Called by Settings.Slide.Reset and Reset2 in the SlideLimits
        %uipanel
        switch source.String
            case 'this'
                val = Settings.Slide.Menu.Value;
                Data.Slide(val).Min = Data.Lims(val,1);
                Data.Slide(val).Max = Data.Lims(val,2);
                Settings.Slide.Min(val).String = Data.Slide(val).Min*Check(val);
                Settings.Slide.Max(val).String = Data.Slide(val).Max*Check(val);
                Data.Slide(val).Value = 0;
                Data.SlideText(val).String = 0;
            case 'all'
                for i = 1:5
                    Data.Slide(i).Min = Data.Lims(i,1);
                    Data.Slide(i).Max = Data.Lims(i,2);
                    Settings.Slide.Min(i).String = Data.Slide(i).Min*Check(i);
                    Settings.Slide.Max(i).String = Data.Slide(i).Max*Check(i);
                end
                ResetArms_Callback(Data.ResetArm);
        end
    end

%%Saves data to disk
    function Save_Callback(source,~)
        switch source.Parent.Title
            case 'Pin Settings'
                if Data.Set
                    File = Data.Pins(:,:);
                    uisave('File','Pins.mat')
                    File = [];
                else
                    Settings.StatusDisplay.String = 'Please set pins first';
                end
            case 'Calibration'                
                if PinCheck
                    File = {Data.PLims(:,:);Data.PSet};                    
                    uisave('File','Calibration.mat')
                    File = [];
                else
                    Settings.StatusDisplay.String = 'Please calibrate pins first';
                end
            case 'Open Loop Automation'
            case 'Closed Loop Automation'
                if numel(Program.Config)
                    File = Program.Config;
                    uisave('File','Movement')
                    File = [];
                else
                    Settings.StatusDisplay.String = 'Start a motion first';
                end
        end
    end

%%Sets the Pin ports for the arduino
    function SetPin_Callback(~,~)        
        %Called by Settings.Cal.Set in the PinSettings uipanel
        Settings.StatusDisplay.String = '';
        if ArduinoDetected
            c = 0;
            for i = 1:5
                if isempty(Settings.Pin.APin(i).String)
                    Status(sprintf('Input  M%d''s Analog  pin Value',6-i))
                    c = 1;
                end
                if isempty(Settings.Pin.Pos(i).String)
                    Status(sprintf('Input  M%d''s Positive pin Value',6-i))
                    c = 1;
                end
                if isempty(Settings.Pin.Neg(i).String)
                    Status(sprintf('Input  M%d''s Negative  pin Value',6-i))
                    c = 1;
                end
                if c
                    continue
                end
                Data.Pins(i,1) = eval(Settings.Pin.APin(i).String);
                Data.Pins(i,2) = eval(Settings.Pin.Neg(i).String);
                Data.Pins(i,3) = eval(Settings.Pin.Pos(i).String);
            end
            Data.Pins(6,1) = Settings.Pin.High.Value;
            Data.Pins(6,2) = Settings.Pin.Low.Value;
            if c || ~CheckPinValues()
                Status(sprintf('Check Pins'))
                Data.Set = 0;
            else
                Status(sprintf('Values Check Out'))
                Data.Set = 1;
                CalSettings.SelectedObject = Settings.Cal.Button(1);
                CalButton_Callback(Settings.Cal.Button(1));
                CalPanel.Visible = 'on';
                for i = 1:5
                    digitalWrite(Data.Pins(i,2),~Data.Pins(6,1));
                    digitalWrite(Data.Pins(i,3),~Data.Pins(6,1));
                end
            end
        else
            Settings.StatusDisplay.String = 'Arduino board not passed to GUI';
        end
    end

%%Changes the configuration of the robot arm
    function Slide_Callback(source,~)        
        %Called by Data.Slide<> in the ViewTab
        index = source.UserData;
        Data.SlideText(index).String = num2str(source.Value*Check(index),3);
        Update
    end

%%Sets the limit of the chosen slide
    function SlideLimit_Callback(source,~)        
        %Called by Settings.Slide.Label in the SlideSettings uipanel
        val = source.UserData;
        if ~isempty(Settings.Slide.Min(str2double(val)).String)&&~isempty(Settings.Slide.Max(str2double(val)).String)
            eval(strcat('Data.Slide(',val,').Min = eval(Settings.Slide.Min(',val,').String)/Check(',val,');'));
            eval(strcat('Data.Slide(',val,').Max = eval(Settings.Slide.Max(',val,').String)/Check(',val,');'));
            eval(strcat('Data.Slide(',val,').Value = eval(Settings.Slide.Max(',val,').String)/Check(',val,');'));
            eval(strcat('Data.SlideText(',val,').String = eval(Settings.Slide.Max(',val,').String);'));
            Update %eg ,if val is 1; Data.Slide(1).Value = Settings.Slide.Min1.String;
        end
    end

%%Sets slide values
    function SlideText_Callback(source,~)
        %Called by Data.SlideText<> in the ViewTab
        index = source.UserData;
        Data.Slide(index).Value = str2double(source.String)/Check(index);
        Update
    end

%Allows the status display and remote to span multiple tabs
    function TabChanged_Callback(source,event)
        %Called whenever the user switches tabs
        if strcmp(source.Units,'normalized')
            xmax1 = 280;
            ymax1 = 500;
            xmax2 = 252;
            ymax2 = 200;
        else
            xmax1 = 1;
            xmax2 = xmax1;
            ymax1 = 1;
            ymax2 = ymax1;
        end
        switch event.NewValue.Title
            case 'Arduino'
                RemoteSettings.Parent = PinSettings;
                RemoteSettings.Position = [110/xmax2 22/ymax2 26/xmax2 130/ymax2];
                Settings.StatusDisplay.Parent = ArduinoTab;
                Settings.StatusDisplay.String = '';
                Data.Stop.Position = [662.5/750,125/500,70/750,80/500];
            case 'Display'
                Settings.StatusDisplay.Parent = ViewTab;
                RemoteSettings.Parent = ViewTab;
                RemoteSettings.Position = [2/xmax1 345/ymax1 26/xmax1 152/ymax1];
                Settings.StatusDisplay.String = '';
                if Settings.Advanced1.Value || Settings.Advanced2.Value
                    Data.Stop.Position = [662.5/750,12/500,70/750,80/500];
                else
                    Data.Stop.Position = [662.5/750,125/500,70/750,80/500];
                end
            case 'Automation'
                RemoteSettings.Parent = ClosedLoopPanel;
                RemoteSettings.Position = [10/xmax1 253/ymax1 35/xmax1 245/ymax1];
                Settings.StatusDisplay.Parent = ProgramTab;
                Settings.StatusDisplay.String = '';
                Data.Stop.Position = [662.5/750,125/500,70/750,80/500];                
        end
    end

%%Uploads one or more arm configurations to the arduino
    function Upload_Callback(source,~)        
        %Called by Data.Upload<> and Data.UploadAll in the ViewTab
        val = source.UserData;
        if PinCheck
            if val < 6
                move(val);
            else
                for i = 1:5
                    Upload_Callback(Data.Upload(6-i))
                end
            end
        elseif ~Data.Set
            Settings.StatusDisplay.String = 'Set Pins first';
        else
            Status('Head to the Arduino Tab');
        end
    end

%%changes the Plot view
    function View_Callback(source,~)        
        %called by all buttons in the ViewSettings menu in the ViewTab
        switch source.String
            case 'YX'
                view(-90,90);
            case 'XZ'
                view(0,0);
            case 'XZ'''
                view(-180,0);
            case 'YZ'
                view(90,0);
            case 'YZ'''
                view(-90,0);
            case 'RESET'
                view(-150,30);
            case 'ISO'
                [az,~]=view;
                az = az/45 - mod(az,45)/45;
                if -1 <= az && az < 1
                    view(-45,30);
                elseif 1 <= az && az <3
                    view(45,30);
                elseif 3 <= az && az <5 ||-5< az && az <-3
                    view(135,30);
                else
                    view(225,30);
                end
            case 'ROTATE'
                if source.Value
                    rotate3d on
                else
                    rotate3d off
                end
        end
    end

%%Displays the slide limit edit boxes indicated by the menu
    function ViewSlider_Callback(source,~)
        %Called by Settings.Slide.ShowBoxes in the ViewTab
        val = Settings.Slide.Menu.Value;
        if source.Value
            Settings.Slide.TopLabel.Visible = 'on';
            eval(strcat('Settings.Slide.Min(',num2str(val),').Visible = ''on'';'));
            eval(strcat('Settings.Slide.Max(',num2str(val),').Visible = ''on'';'));
            eval(strcat('Settings.Slide.Label(',num2str(val),').Visible = ''on'';'));
            Settings.Slide.Menu.Enable = 'off';
        else
            Settings.Slide.TopLabel.Visible = 'off';
            eval(strcat('Settings.Slide.Min(',num2str(val),').Visible = ''off'';'));
            eval(strcat('Settings.Slide.Max(',num2str(val),').Visible = ''off'';'));
            eval(strcat('Settings.Slide.Label(',num2str(val),').Visible = ''off'';'));
            Settings.Slide.Menu.Enable = 'on';
        end
    end

%% FUNCTIONS

%EMERGENCY STOP
    function Stop(~,~)
        if Data.Set 
            for i = 1:5
%                 P_Pin = strcat('D',num2str(Data.Pins(i,3)));
                %                 Arduino.configurePin(P_Pin,'unset');
                digitalWrite(Data.Pins(i,3), ~Data.Pins(6,1))
%                 N_Pin = strcat('D',num2str(Data.Pins(i,2)));
                %                 Arduino.configurePin(N_Pin,'unset');
                digitalWrite(Data.Pins(i,2), ~Data.Pins(6,1))
            end
        else
            Status('-_- Stop what?');
        end
    end

%MAKES BUTTON SETS
    function ButtonSet(val,Base,DataStruct,Values,Parent,state,bracket)
        %example x,[10 10], Settings, Value array, Axis, 'on', 'on'
        if strcmp(bracket,'on')
            bo = '('; bc = ')';
        else
            bo = ''; bc='';
        end
        front = strcat(DataStruct,'.',Parent);
        middle1 = '= uicontrol(''Style'',''';
        middle2 = strcat(''',''Parent'',',Parent,'Settings,''Visible'',''',state,''',''UserData'',''',val,''',');
        for i = 1:size(Values,1)
            statement = strcat(front,'.',Values{i,1},bo,val,bc,middle1,Values{i,2},middle2);
            %Settings.Axis.<values,i,1> =
            %uicontrol('Style','<values(i,2)>','Parent',AxisSettings,'Visible','on','UserData',<val>,
            if Values{i,4} ~= Inf
                statement = strcat(statement,'''String'',''',Values{i,4},'',val,''',');
                %adds on : 'String',<values(i,4),>
            end
            if Values{i,5} ~= Inf
                statement = strcat(statement,'''Callback'',@',Parent,Values{i,5},',');
                %adds on : 'Callback',@Axis<values(i,5),>
            end
            statement = strcat(statement,'''Position'',[');
            %adds on : 'Position',[
            if i == 1
                Position = [Base(1:2) 0 0 ]+[0 0 Values{i,3}];
            else
                pad = Values{i-1,3};
                Position = [Base(1:2) 0 0 ]+[0 0 Values{i,3}]+[(i-1)*(Base(1,3)+pad(1,1)) 0 0 0];
            end
            eval(strcat(statement,num2str(Position),']);'));
        end
    end

%CONVERT FROM RAD TO DEGRESS IF RAD BUTTON IS PUSHED
    function result = Check(index)
        if strcmp(Data.Slide(index).String,'R') && ~Data.Radians.Value
            result = 180/pi;
        else
            result = 1;
        end
    end

% CHECK FOR LEGAL PIN VALUES
    function result = CheckPinValues(varargin)
        %make sure pins are within the right range
        %make sure pins are integers
        count = 0;
        for i = 1:5
            pos = Data.Pins(i,3);
            neg = Data.Pins(i,2);
            apin = Data.Pins(i,1);
            if mod(pos,1)~=0
                Status(sprintf('M%d''s +ive pin Value is Not an Integer',6-i));
                count = count+1;
            end
            if mod(neg,1) ~=0
                Status(sprintf('M%d''s -ive pin Value is Not an Integer',6-i));
                count = count+1;
            end
            if mod(apin,1)~=0
                Status(sprintf('M%d''s Analog pin Value an Not an Integer',6-i));
                count = count+1;
            end
            if pos>54 || pos< 2
                Status(sprintf('M%d''s +ive pin Value is out of range',6-i));
                count = count+1;
            end
            if neg>54 || neg< 2
                Status(sprintf('M%d''s -ive pin Value is out of range',6-i));
                count = count+1;
            end
            if apin>16 || apin< 0
                Status(sprintf('M%d''s Analog pin Value is out of range',6-i));
                count = count+1;
            end
            if count
                break
            end
        end
        if  RepeatedPins() || count %make sure pins are not repeated
            result = 0;
        else
            result = 1;
        end
    end

%PRINT STATUS MESSAGE TO STATUS DISPLAY
    function Status(string)
        existing = Settings.StatusDisplay.String;
        additional = cell(1);
        additional{1} = string;
        new = [existing;additional];
        Settings.StatusDisplay.String = new;
    end

%CHECK IF ANY DIGITAL OR ANALONG PINS ARE REPEATED
    function result = RepeatedPins()
        checkr = 0;
        %check analog column for repeated values
        for i = 1:4
            checkval = Data.Pins(i,1);
            for k = i:4
                if checkval == Data.Pins(k+1,1)
                    Status(sprintf('M%d''s Analog pin Value is a duplicate',6-i));
                end
            end
        end
        %check output columns for repeated values
        if ~checkr
            for i = 1:9
                checkval = Data.Pins(ceil(i/2),mod(i,-2)+3);
                for k = i:9
                    if checkval == Data.Pins(ceil((k+1)/2),mod((k+1),-2)+3)
                        if mod((k+1),-2)+3 == 2
                            Status(sprintf('M%d''s Negative pin Value is a duplicate',6-ceil(i/2)));
                        else
                            Status(sprintf('M%d''s Positive pin Valie is a duplicate',6-ceil(i/2)));
                        end
                        checkr = checkr + 1;
                    end
                end
            end
        end
        if checkr
            result = 1;
        else
            result = 0;
        end
    end

%UPDATE THE FIGURE
    function Update()
        for i = 1:4 %for every arm but the gripper, update the transforms
            Data.DH(i,:) = [Data.input(i,1), Data.input(i,2), Data.input(i,3), -Data.Slide(i).Value];
            if i == 1
                Data.HGTrans(i).Matrix  = A(Data.DH(i,:));
            else
                Data.HGTrans(i).Matrix  = Data.HGTrans(i-1).Matrix * A(Data.DH(i,:));
            end
        end
        %gripper transforms
        %pi/90 constant converts a full scale value of 45mm to a pi/2 degree turn
        Data.HGTrans(05).Matrix = Data.HGTrans(4).Matrix * A([   0, pi/2,  12.1, 0]) * A([0,0,0, Data.Slide(5).Value * pi/90]);
        Data.HGTrans(06).Matrix = Data.HGTrans(4).Matrix * A([   0, pi/2, -12.1, 0]) * A([0,0,0,-Data.Slide(5).Value * pi/90]);
        Data.HGTrans(07).Matrix = Data.HGTrans(4).Matrix * A([16.8, pi/2,   6.3, 0]) * A([0,0,0, Data.Slide(5).Value * pi/90]);
        Data.HGTrans(08).Matrix = Data.HGTrans(4).Matrix * A([16.8, pi/2,  -6.3, 0]) * A([0,0,0,-Data.Slide(5).Value * pi/90]);
        Data.HGTrans(09).Matrix = Data.HGTrans(7).Matrix * A([  23,    0,     0, 0]) * A([0,0,0,-Data.Slide(5).Value * pi/90]);
        Data.HGTrans(10).Matrix = Data.HGTrans(8).Matrix * A([  23,    0,     0, 0]) * A([0,0,0, Data.Slide(5).Value * pi/90]);
    end

%MAKE TRANSFORMATION MATRIX
    function result = A(DH)%DH = (r,alpha,d,theta)
        Ct = cos(DH(4));        St = sin(DH(4));
        Sa = sin(DH(2));        Ca = cos(DH(2));
        r = DH(1);        d = DH(3);
        result = [Ct, -St*Ca, St*Sa, r*Ct;...
            St, Ct*Ca, -Ct*Sa, r*St;...
            0, Sa, Ca, d;...
            0, 0, 0, 1];
    end

%ANALOG READ
    function result = analogRead(pin)
        pin = strcat('A',num2str(pin));
        result = Arduino.readVoltage(pin);
    end

%ANALOG WRITE 
    function analogWrite(pin,Value)
        pin = strcat('D',num2str(pin));
        Mega.writePWMVoltage(pin,Value);
    end

%DIGITAL WRITE
    function digitalWrite(pin,Value)
        pin = strcat('D',num2str(pin));
        Arduino.writeDigitalPin(pin,Value);
    end

%SET AN ARM CONFIGURATION (angle values in degrees)
    function Config(input,varargin)
        if nargin == 1
            for i = 1:5
                Data.Slide(i).Value = input(i)/Check(i);
                Slide_Callback(Data.Slide(i));
            end
        else
            SlideConfig = GetConfig(1);
            for i = 1:5
                n = input(6-i) - SlideConfig(6-i);
                m = n - rem(n,1);
                if m
                    for k = 1:abs(m)
                        Data.Slide(6-i).Value = (SlideConfig(6-i)+k*m/abs(m))/Check(6-i);
                        Slide_Callback(Data.Slide(6-i));
                        pause(.01);
                    end
                    if rem(n,1)
                        Data.Slide(6-i).Value = input(6-i)/Check(6-i);
                        Slide_Callback(Data.Slide(6-i));
                    end
                end
            end
        end
    end

%RETURNS EITHER THE CURRENT CONFIG OF THE ACTUAL ROBOT, OR THAT OF THE PLOT
    function result = GetConfig(varargin)
        if nargin
            M5 = Data.Slide(1).Value * 180/pi;
            M4 = Data.Slide(2).Value * 180/pi;
            M3 = Data.Slide(3).Value * 180/pi;
            M2 = Data.Slide(4).Value * 180/pi;
            M1 = Data.Slide(5).Value;
        else
            M5 = map(analogRead(Data.Pins(1,1)),Data.PLims(1,:),Data.Lims(1,:)*Check(1));
            M4 = map(analogRead(Data.Pins(2,1)),Data.PLims(2,:),Data.Lims(2,:)*Check(2));
            M3 = map(analogRead(Data.Pins(3,1)),Data.PLims(3,:),Data.Lims(3,:)*Check(3));
            M2 = map(analogRead(Data.Pins(4,1)),Data.PLims(4,:),Data.Lims(4,:)*Check(4));
            M1 = map(analogRead(Data.Pins(5,1)),Data.PLims(5,:),Data.Lims(5,:));
        end
        result = ([M5 M4 M3 M2 M1]);
    end

%DELAY
    function delay(time)%%at least 10ms
        pause(time/1000);
    end

%MAP FUNCTION
    function result = map(val,angleLims,voltLims)
        x1 = angleLims(1);
        y1 = angleLims(2);
        x2 = voltLims(1);
        y2 = voltLims(2);
        result = (val - x1)*(y2 - x2)/(y1-x1)+ x2;
    end

%CHECK IF ALL PINS HAVE BEEN CALIBRATED
    function result = PinCheck()
        result = 1;
            Settings.StatusDisplay.String = '';
        for val = 1:5
            if ~(strcmp(Data.PSet(val,1),'Set')&&strcmp(Data.PSet(val,2),'Set'))
                Status(strcat('M',num2str(val),' hasn''t been calibrated'));
                result = 0;
                break
            end
        end
    end

%GET DIRECTION TOWARDS GOAL
    function result = getDir(val,varargin)
        if nargin == 1
            angleVoltage = map(Data.Slide(val).Value,Data.Lims(val,:),Data.PLims(val,:));
            angleOffset = abs(angleVoltage - Data.PLims(val,1));
            armOffset = abs(analogRead(Data.Pins(val,1))-Data.PLims(val,1));
            if angleOffset > armOffset + tolerance
                result = 3;
            elseif angleOffset < armOffset - tolerance
                result = 2;
            else
                result = 0;
            end
        else
            goal = varargin{1};
            if isnumeric(goal)
                angleVoltage = map(goal,Data.Lims(val,:),Data.PLims(val,:));
                angleOffset = angleVoltage - Data.PLims(val,1);
                armOffset = analogRead(Data.Pins(val,1))-Data.PLims(val,1);
                if angleOffset > armOffset + tolerance
                    result = 3;
                elseif angleOffset < armOffset - tolerance
                    result = 2;
                else
                    result = 0;
                end
            elseif strcmp(goal,'+')
                result = 3;
            elseif strcmp(goal,'-')
                result = 2;
            else
                result = 0;
            end
        end
    end

%MOVE ARM
    function move(val,varargin)
        if nargin == 1
            Pin = getDir(val);
        elseif nargin > 1
            Pin = getDir(val,varargin{1});
        end
        %         stuck = 0;
        %         count = 0;
        %         switch val
        %             case 1
        %                 hits = 9;
        %             case 2
        %                 hits = 5;
        %             case 3
        %                 hits = 5;
        %             case 4
        %                 hits = 3;
        %             case 5
        %                 hits = 3;
        %         end
        while (Pin)
            %             start = analogRead(Data.Pins(val,1));
            MovePin = Data.Pins(val,Pin);
            digitalWrite(MovePin,Data.Pins(6,1));
            delay(60);
            digitalWrite(MovePin,~Data.Pins(6,1));
            if nargin == 1
                Pin = getDir(val);
            else
                Pin = getDir(val,varargin{1});
            end
            %             stop = analogRead(Data.Pins(val,1));
            %             if abs(start - stop) < 0.1
            %                 count = count + 1;
            %                 if count == hits
            %                     stuck = 1;
            %                     break
            %                 end
            %             end
        end
        %         if stuck
        %             Settings.StatusDisplay.String = 'boop';
        %         end
    end

%AUTOCALIBRATE FUNCTION shafted due to physical limitations
%     function AutoCalibrate()
%         clc
%         for i = [1 4 5]
%             move(i,'-')
%             Data.PLims(i,1) = analogRead(Data.Pins(i));
%             move(i,'+')
%             Data.PLims(i,2) = analogRead(Data.Pins(i));
%             move(i,0)
%         end
%         move(4,'-')
%         move(3,'+');
%         %set
%         Data.PLims(3,2) = analogRead(Data.Pins(3));
%         move(2,'-');
%         %set
%         Data.PLims(2,1) = analogRead(Data.Pins(2));
%         move(2,'+');
%         move(3,'-');
%         move(2,'+');
%         Data.PLims(2,2) = analogRead(Data.Pins(2));
%         %set
%         move(2,'+');
%         move(3,'-');
%         %set
%         Data.PLims(3,1) = analogRead(Data.Pins(3));
%         move(4,0);
%         move(2,0);
%         move(3,0);%
%     end

end