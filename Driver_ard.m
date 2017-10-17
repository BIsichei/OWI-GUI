function Driver_ard(Arduino)
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
    
    
    TODO:
    Move function
    Speed Control
    Stop
    Save and Load
    Automation
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
dim = 'xyz';
%Set GUI to only work for Mega
ArduinoDetected = nargin && ~isempty(findobj(Arduino,'Board','Mega2560'));
tolerance = 0.2; %volts
Interrupt = 0;
saveindex = 1;
Active = 1;

File = struct; % temp data storage for saving and loading
Data = struct;
Pins = struct;
Settings = struct;
Program = struct;

Data.input = [0, -pi/2, 0, Inf;
    90, 0, 0, Inf;
    118, 0, 0, Inf;
    41, 0, 0, Inf];
Data.ArmLimits  = [-3*pi/4, 3*pi/4;
    0, pi;
    -5*pi/6, 5*pi/6;
    -pi/3, pi/3;
    0, 45];
Pins.Default = [     0     5     4;
     1     6     7;
     2     8     9;
     3     10    11;
     4     13    12;
     0     1     0];
Pins.Set = 0;
for j = 1:5
    Pins.Ana(j) = NaN;
    Pins.En(j) = NaN;
    Pins.Dir(j) = NaN;
    Pins.Pot.Min(j).Value = NaN;
    Pins.Pot.Min(j).Set = 0;
    Pins.Pot.Max(j).Value = NaN;
    Pins.Pot.Max(j).Set = 0;
end
Program.Config = {};

Data.Stop = uicontrol('Position',[662.5,125,70,80],'String','STOP',...
    'BackgroundColor',[.8 .3 .3],'Callback',@Stop);
Data.RevUpdate = uicontrol('String','UpdatePlot',...
    'Callback',@PlotUpdate_Callback);

%% VIEW TAB SETUP
ViewSettings = uipanel(ViewTab,'Units','Pixels','Position',[10,230,155,105],...
    'Title', 'VIEWS','TitlePosition','centertop');
AxisSettings = uipanel(ViewTab,'Units','Pixels','Position',[10,10,180,115],...
    'Title','AXIS LIMITS','TitlePosition','centertop','Visible','off');
SlideSettings = uipanel(ViewTab,'Units','Pixels','Position',[10,10,180,115],...
    'Title','SLIDE LIMITS','TitlePosition','centertop','Visible','off');
RemoteSettings = uipanel(ViewTab,'Units','Pixels','Position',[2 345 26 130]);
DHSettings = uipanel(ViewTab,'Units','Pixels','Position',[10,10,180,115],...
    'Title','DH Parameters','TitlePosition','centertop','Visible','off');


%VIEWTAB CHILDREN
base = 350; %base y value for sliders
for j = 1:5
    Data.SlideReset(j) = uicontrol(ViewTab,'UserData',j,...
        'Position',[32,base+30*(j-1)+10,20,12],'Callback',@ResetArms_Callback,...
        'String',strcat('M',num2str(6-j)),'TooltipString','Reset this joint');
    Data.SlideText(j) = uicontrol(ViewTab,'Style','edit','UserData',j,...
        'Position',[32,base+30*(j-1)-2,20,14],'Callback',@SlideText_Callback,...
        'String',0);
    Data.PowerSlide(j) = uicontrol(ViewTab,'Style','Slider','UserData',j,...
        'Position',[245,base+30*(j-1)+10,5,12],'Callback',@Slide_Callback,...
        'String','P','Min',0,'Max',200,'Value',100);
    Data.PowerText(j) = uicontrol(ViewTab,'Style','edit','UserData',j,...
        'Position', [56,base+30*(j-1)+10,20,12],'Callback',@PowerText_Callback,...
        'String',100);
    Data.Slide(j) =  uicontrol(ViewTab,'Style','slider','UserData',j,...
        'Position',[55,base+30*(j-1),200,10],'Callback',@Slide_Callback,...
        'String', 'R','Min', Data.ArmLimits(j,1),'Max',Data.ArmLimits(j,2),...
        'Value',0);
    Data.Upload(j) = uicontrol(ViewTab,'FontSize',12,'UserData',j,...
        'position',[258,base+30*(j-1)-2,14,12],'Callback',@Upload_Callback,...
        'String','>','FontWeight','bold','TooltipString','Upload to board');        
    align([Data.SlideText(j) Data.SlideReset(j)],'Center','Fixed',1);
    align([Data.SlideText(j) Data.Slide(j) Data.Upload(j)],'None','Middle');
    align([Data.PowerText(j) Data.Upload(j)],'Right','Fixed',1);
    align([Data.PowerSlide(j) Data.PowerText(j)],'None','Top');
end
for j = 1:size(Data.input,1)
    Data.HGTrans(j) = hgtransform(Axes1);
end
for j = 1:6
    Data.GripTrans(j) = hgtransform(Axes1);
end
Data.Slide(5).SliderStep = [1/90 1/18];
Data.Slide(5).String = 'G';
Data.UploadAll = uicontrol(ViewTab,'Position',[175,290,90,39],...
    'UserData',[1 2 3 4 5],'String','Upload all',...
    'Callback',@Upload_Callback,'TooltipString','Upload all Values');
Data.ResetArm = uicontrol(ViewTab,...
    'Position',[175,260,90,20],'String','Reset Arm',...
    'Callback',@ResetArms_Callback,'UserData',6);
Data.Radians = uicontrol(ViewTab,'Position', [175,230,90,20],...
    'String','Radians','Style','toggle','Callback',@Radian_Callback);

Settings.StatusDisplay = uicontrol(ViewTab,'Style','text',...
    'Position',[10,125,180,95],'Max',2,'HorizontalAlignment','left',...
    'BackgroundColor',[.91 .91 .93]);
Settings.Axis.Reset = uicontrol(ViewTab,'Position',[195 180 70 40],...
    'String','Reset Axis','Callback',@ResetAxis_Callback,...
    'Visible','off');
Settings.Slide.Reset = uicontrol(ViewTab,'UserData','all',...
    'Position',[195 180 70 40],'Callback',@ResetLimits_Callback,...
    'String','Reset all Slides','Visible','off');
    
Settings.Advanced1 = uicontrol(ViewTab,'Style','radio',...
    'String','Axis','Position',[5 5 50 20],...
    'UserData','Axis','Callback',@AdvancedControls_Callback);
Settings.Advanced2 = uicontrol(ViewTab,'Style','radio',...
    'String','Slide','Position',[30 5 50 20],...
    'UserData','Slide','Callback',@AdvancedControls_Callback);
Settings.DH = uicontrol(ViewTab,'Style','radio',...
    'String','Show DH','Position',[55 5 50 20],...
    'UserData','DH','Callback',@AdvancedControls_Callback);


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
for j = 1:3
    Settings.Axis.Min(j) = uicontrol(AxisSettings,'Style','edit','Userdata',j,...
        'Position',[10,10+(j-1)*30,50,20]);
    Settings.Axis.Max(j) = uicontrol(AxisSettings,'Style','edit','Userdata',j,...
        'Position',[65,10+(j-1)*30,50,20]);
    Settings.Axis.Set(j) = uicontrol(AxisSettings,'Userdata',j,...
        'Position',[120,10+(j-1)*30,50,20],'Callback',@AxisLimit_Callback,...
        'String', strcat('Set ',dim(j)));
end


%SLIDESETTINGS CHILDREN
Settings.Slide.Menu = uicontrol(SlideSettings,'Style','pop',...
    'Position',[10,60,50,20],'String',{'M5';'M4';'M3';'M2';'M1'});
Settings.Slide.ShowBoxes = uicontrol(SlideSettings,'Style','toggle',...
    'Position',[70,56,100,25],'String','View Slide Limit Boxes',...
    'Callback',@ViewSlider_Callback);
Settings.Slide.TopLabel = uicontrol(SlideSettings,'Style','text',...
    'String','Min : Max','Position',[10,23,40,20],'Visible','off');
Settings.Slide.Reset2 = uicontrol(SlideSettings,'Userdata','single',...
    'Position',[70,35,100,15],'String','Reset Slide',...
    'Callback',@ResetLimits_Callback);
for j = 1:5
    Settings.Slide.Min(j) = uicontrol(SlideSettings,'Style','edit','UserData',j,...
        'Position',[10,10,50,20],'Visible','off');
    Settings.Slide.Max(j) = uicontrol(SlideSettings,'Style','edit','UserData',j,...
        'Position',[65,10,50,20],'Visible','off');
    Settings.Slide.Label(j) = uicontrol(SlideSettings,'UserData',j,...
        'Position',[120,10,50,20],'Callback',@SlideLimit_Callback,...
        'String','Limit Slider','Visible','off');
end

%DH CHILDREN
Data.DHTable = uitable(DHSettings,'Data',[Data.input(:,1:3) [Data.Slide(1).Value;Data.Slide(2).Value;Data.Slide(3).Value;Data.Slide(4).Value]],...
    'Position',[4,20,170,64],'RowName',{'M2','M3','M4','M5'},...
    'ColumnName',{'R','alpha','d','theta'},'ColumnWidth',{66,68,66,69});



%REMOTESETTINGS CHILDREN
for j = 1:5
    Settings.Remote(j) = uicontrol(RemoteSettings,'Userdata',j,...
        'Position',[2,3+25*(j-1),20,20],'Callback',@Remote_Callback,...
        'String',strcat('M',num2str(6-j)),'BackgroundColor',[.9 .9 1],...
        'KeyPressFcn',@RemoteKeyPress_Callback,'ForegroundColor',[.5 .6 .7]);
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

Settings.Pin.EnableLabel = uicontrol(PinSettings,'Style','text',...
    'Position',[155,125,15,15],'String','En',...
    'TooltipString','These values indicate what pins enable the motors');
Settings.Pin.DirectionLabel = uicontrol(PinSettings,'Style','text',...
    'Position',[176,125,15,15],'String','Dir',...
    'TooltipString','These values indicate what direction the motors move');
base = 25;
offset = 22;
for j = 1:5
    Settings.Pin.ALabel(j) = uicontrol(PinSettings,'Style','text','UserData',j,...
        'Position',[5,base+offset*(j-1),45,15],'HorizontalAlignment','left',...
        'String',strcat('M',num2str(6-j),' Pot Pin'));
    Settings.Pin.APin(j) = uicontrol(PinSettings,'Style','edit','UserData',j,...
        'Position',[23,base+offset*(j-1),20,15]);
    Settings.Pin.En(j) = uicontrol(PinSettings,'Style','edit','UserData',j,...
        'Position',[155,base+offset*(j-1),20,15]);
    Settings.Pin.Dir(j) = uicontrol(PinSettings,'Style','edit','UserData',j,...
        'Position',[176,base+offset*(j-1),20,15]);
    Settings.Pin.Label(j) = uicontrol(PinSettings,'Style','text','UserData',j,...
        'Position',[36,base+offset*(j-1),65,15],'HorizontalAlignment','Right',...
        'String',strcat('M',num2str(6-j),' Output Pins'));
    align ([Settings.Pin.ALabel(j) Settings.Pin.APin(j) Settings.Pin.En(j) ...
        Settings.Pin.Dir(j) Settings.Pin.Label(j)], 'Fixed',1,'Middle');    
end
Settings.Pin.InterruptLabel = uicontrol(PinSettings,'Style','text','UserData',7,...
        'Position',[5,base+offset*(5),45,15],'HorizontalAlignment','Left',...
        'String','Interrupt');
Settings.Pin.Interrupt = uicontrol(PinSettings,'Style','edit','UserData',7,...
        'Position',[36,base+offset*(5),20,15]);
Settings.Pin.LightLabel = uicontrol(PinSettings,'Style','text','UserData',6,...
        'Position',[77,base+offset*(5),40,15],'HorizontalAlignment','Left',...
        'String','Light Pin');
Settings.Pin.Light = uicontrol(PinSettings,'Style','edit','UserData',6,...
        'Position',[120,base+offset*(5),20,15]);
align([Settings.Pin.EnableLabel Settings.Pin.Label(5)],'Fixed',2,'Fixed',1);
align([Settings.Pin.DirectionLabel Settings.Pin.EnableLabel],'Fixed',2,'Middle');
align([Settings.Pin.LightLabel Settings.Pin.Light],'Fixed',1,'Bottom');
align([Settings.Pin.InterruptLabel Settings.Pin.Interrupt], 'Fixed',1,'Botton');

Settings.Pin.Set = uicontrol(PinSettings,'String','Apply All Values',...
    'Position',[5,3,260,20],'Callback', @SetPin_Callback);    
Settings.Pin.Save = uicontrol(PinSettings,'Position',[200 170 50 20],...
    'Callback',@Save_Callback,'String','Save Pins');
Settings.Pin.Load = uicontrol(PinSettings,'Position',[200 145 50 20],...
    'Callback',@Load_Callback,'String','Load Pins');


%CALPANEL CHILDREN
CalSettings = uibuttongroup(CalPanel,'Units','Pixels','Position',[0 188 270 50]);
for j = 1:5
    Settings.Cal.Button(j) = uicontrol(CalSettings,'Style','toggle','UserData',j,...
        'Position',[1+53*(j-1),5,50,20],'Callback',@CalButton_Callback,...
        'String',strcat('M',num2str(6-j)));
end
Settings.Cal.Orient = uicontrol(CalPanel,'Units','Pixels','String','Orient',...
    'Position',[5 30 30 20],'Callback',@Orientation_Callback);
Settings.Cal.Set = uicontrol (CalPanel,'Units','Pixels',...
    'Position',[40 30 30 20],'String','Set','Callback',@Calibrate_Callback);
Settings.Cal.ReadingLabel = uicontrol(CalPanel,'Style','text','Units','Pixels',...
    'Position',[80 30 45 20],'String','Reading:');
Settings.Cal.Reading = uicontrol(CalPanel,'Style','text','Units','Pixels',...
    'Position',[125 30 50 20],'String','--','BackgroundColor',[.98 .98 .98]);
Settings.Cal.Save = uicontrol(CalPanel,'Position',[190 30 70 20],...
    'Callback',@Save_Callback,'String','Save Calibration');
Settings.Cal.Load = uicontrol(CalPanel,'Position',[190 5 70 20],...
    'Callback',@Load_Callback,'String','Load Calibration');

%% PROGRAM TAB SETUP
ClosedLoopPanel = uipanel(ProgramTab,'Title','Closed Loop Automation');
% OpenLoopPanel = uipanel(ProgramTab,'Visible','off','Title','Open Loop Automation');
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
    'UserData',[1 2 3 4 5],'String','Upload Motion',...
    'Callback',@ProgramMove_Callback,'Tag','Upload');


%OPEN LOOP CHILDREN

%% DRAWING ITEMS
y = [231 188 34]./255;
%Base1
[Vertices, Faces , ~, ~]= stlReadBinary('Models/Base_1.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'FaceColor',y,'Parent',Data.HGTrans(1),'EdgeColor',y/2);
%Base2
[Vertices, Faces , ~, ~]= stlReadBinary('Models/Base_2.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'FaceColor',[.1 .1 .1],'Parent',Data.HGTrans(1));
% Arm1
[Vertices, Faces , ~, ~]= stlReadBinary('Models/arm1.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.HGTrans(2),'FaceColor',[.1 .1 .1]);
%Arm2_1
[Vertices, Faces, ~, ~]= stlReadBinary('Models/arm2_1.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'FaceColor',[.1 .1 .1],'Parent',Data.HGTrans(3));
%Arm2_2
[Vertices, Faces, ~, ~]= stlReadBinary('Models/arm2_2.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'FaceColor',y,'Parent',Data.HGTrans(3),'EdgeColor',y/2);
%Gripper
[Vertices, Faces, ~, ~]= stlReadBinary('Models/gripper.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.HGTrans(4),'FaceColor',[.1 .1 .1]);
%Gripper_1  
[Vertices, Faces, ~, ~]= stlReadBinary('Models/gripper_1.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.GripTrans(1),'FaceColor',[.19 .19 .19]);
%Gripper_2
[Vertices, Faces, ~, ~]= stlReadBinary('Models/gripper_2.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.GripTrans(2),'FaceColor',[.19 .19 .19]);
%Gripper_3 and 4
[Vertices, Faces, ~, ~]= stlReadBinary('Models/gripper_3.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.GripTrans(3),'FaceColor',[.19 .19 .19]);
[Vertices, Faces, ~, ~]= stlReadBinary('Models/gripper_3.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.GripTrans(4),'FaceColor',[.19 .19 .19]);
%Gripper_5 and 6
[Vertices, Faces, ~, ~]= stlReadBinary('Models/gripper_4l.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.GripTrans(5),'FaceColor',[.19 .19 .19]);
[Vertices, Faces, ~, ~]= stlReadBinary('Models/gripper_4r.stl');
patch('Faces',Faces,'Vertices',Vertices,...
    'Parent',Data.GripTrans(6),'FaceColor',[.19 .19 .19]);
%Battery
[Vertices, Faces, ~, ~]= stlReadBinary('Models/battery.stl');
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
g = findobj('Type','uicontrol','-or','Type','uibuttongroup','-or','Type','uipanel','-or','Type','uitable');
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
Settings.DH.FontUnits = 'points';
ResetAxis_Callback
ResetLimits_Callback(Settings.Slide.Reset);
Update(Data)
f.Visible = 'on'; %show the figure

for j = 1:5
    Settings.Pin.APin(j).String = Pins.Default(j,1);
    Settings.Pin.En(j).String = Pins.Default(j,2);
    Settings.Pin.Dir(j).String = Pins.Default(j,3);
    Data.SlideText(j).FontUnits = 'points';
    Data.PowerText(j).FontUnits = 'points';
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
                    Settings.DH.Visible = 'off';
                case 'Slide'
                    SlideSettings.Visible = 'on';
                    Settings.Slide.Reset.Visible = 'on';
                    align([Data.Radians source Settings.Slide.Reset],'Right','None')
                    Settings.Advanced1.Visible = 'off';
                    Settings.DH.Visible = 'off';
                case 'DH'
                    DHSettings.Visible = 'on';
                    Settings.Advanced1.Visible = 'off';
                    Settings.Advanced2.Visible = 'off';
            end
            source.Position = [195/xmax 97/ymax 70/xmax 80/ymax];%[195 97 80 80];
            align([Data.Radians source],'Right','None')
            Data.Stop.Position = [662.5/750,12/500,70/750,80/500];
            source.Style = 'toggle';
            source.String = 'Close Panel';
        else
            DHSettings.Visible = 'off';
            AxisSettings.Visible = 'off';
            SlideSettings.Visible = 'off';
            Settings.Axis.Reset.Visible = 'off';
            Settings.Slide.Reset.Visible = 'off';
            Settings.Advanced1.Visible = 'on';
            Settings.Advanced2.Visible = 'on';
            Settings.DH.Visible = 'on';
            if strcmp(source.UserData,'Axis')
                source.Position = [ 5/xmax 5/ymax 50/xmax 20/ymax];
            source.String ='Axis';
            elseif strcmp(source.UserData,'Slide')
                source.Position = [30/xmax 5/ymax 50/xmax 20/ymax];
            source.String ='Slide';
            else
                source.Position = [55/xmax 5/ymax 50/xmax 20/ymax];
                source.String = 'DH';
            end
            source.Style = 'radio';
            Data.Stop.Position = [662.5/750,125/500,70/750,80/500];
        end
    end
    
    %%sets the x, y or z limits.
    function AxisLimit_Callback(source,~)
        %Called by Settings.Axis.Set<> in the AxisLimit uipanel
        val = source.UserData;
        min = str2double(Settings.Axis.Min(val).String);
        max = str2double(Settings.Axis.Max(val).String);
        switch val
            case 1
                xlim([min max]);
            case 2 
                ylim([min max]);
            case 3
                zlim([min max]);
        end
    end
    
    %%Opens the Calibration Menu for the selected Motor
    function CalButton_Callback(source,~)
        %Called by Settings.Cal.Button in the CalSettings uipanel
        val = source.UserData;
        if ~(Pins.Pot.Min(val).Set && Pins.Pot.Max(val).Set)
            Pins.Pot.Min(val).Set = 0;
            Pins.Pot.Max(val).Set = 0;
            Settings.StatusDisplay.String = strcat('Click Orient to Start Calibrating M',num2str(6-val));
            Settings.Cal.Reading.String = '--';
            Settings.Cal.Set.Visible = 'off';
            Settings.Cal.Orient.Visible = 'on';
            Settings.Cal.Reading.String = analogRead(Arduino,Pins.Ana(val));
        else
            iscalibrated = strcat('M',num2str(6-val),' has been calibrated already. click Reset to recalibrate, or select a different Motor to calibrate');
            Settings.StatusDisplay.String = iscalibrated;
            Settings.Cal.Set.Visible = 'on';
            Settings.Cal.Orient.Visible = 'off';
            Settings.Cal.Set.String = 'Reset';
            Settings.Cal.Reading.String = analogRead(Arduino,Pins.Ana(val));
        end
    end
    
    %%Sets the Min or Max value for the selected motor
    function Calibrate_Callback(source,~)
        %Called by Settings.Cal.Set on the CalSettings uipanel
        val = CalSettings.SelectedObject.UserData;
        Motor = strcat('M',num2str(6-val));
        if ~Pins.Pot.Min(val).Set
            Pins.Pot.Min(val).Value = analogRead(Arduino,Pins.Ana(val));
            Pins.Pot.Min(val).Set = 1;
            Settings.Cal.Reading.String = Pins.Pot.Min(val).Value;
            Settings.StatusDisplay.String = strcat(Motor,' Min has been Set, Click Orient to proceed');
        elseif ~Pins.Pot.Max(val).Set
            Pins.Pot.Max(val).Value = analogRead(Arduino,Pins.Ana(val));
            Pins.Pot.Max(val).Set = 1;
            Settings.Cal.Reading.String = Pins.Pot.Max(val).Value;
            Settings.StatusDisplay.String = strcat(Motor,' Max been calibrated. Click Reset to recalibrate?',Motor,', or select a different Motor to calibrate');
            source.String = 'Reset';
            Settings.Cal.Orient.Visible = 'off';
        else
            Pins.Pot.Min(val).Set = 0;
            Pins.Pot.Max(val).Set = 0;
            source.String = 'Set';
            Settings.Cal.Orient.Visible = 'on';
            Settings.StatusDisplay.String = strcat('Click Orient to begin calibration');
        end
    end       

    %%Loads File from disk
    function Load_Callback(source,~)
        switch source.Parent.Title
            case 'Pin Settings'
                uiopen('Settings/Pins/Pins.mat');
                if numel(File) == 4
                    for i = 1:5
                        Settings.Pin.APin(i).String = File{1}.Ana(i);
                        Settings.Pin.En(i).String = File{1}.En(i);
                        Settings.Pin.Dir(i).String = File{1}.Dir(i);                        
                    end
                    Settings.Pin.Light.String = File{2};
                    Settings.Pin.Interrupt.String = File{3};
                    Settings.Pin.High.Value = File{4};
                    Settings.Pin.Low.Value = ~Settings.Pin.High.Value;
                    File = [];
                    Settings.StatusDisplay.String = 'Pin info loaded';
                else
                    Settings.StatusDisplay.String = 'Invalid file';
                end
            case 'Calibration'
                uiopen('Settings/Cal/Calibration.mat');
                if numel(File) == 1
                    Pins.Pot(:) = File{1};
                    File = [];
                    Settings.StatusDisplay.String = 'Calibration file loaded';
                else
                    Settings.StatusDisplay.String = 'Invalid file';
                end
            case 'Open Loop Automation'
            case 'Closed Loop Automation'
                uiopen('Settings/Move/Movement.mat');
                if size(File,1) == 1
                    Program.Config = {Program.Config{:,:}, File{:,:}};
                    File = [];
                    for i = 1:size(Program.Config,2)
                        Program.List.String{i} = strcat('Config?',num2str(i));
                    end
                    saveindex = size(Program.Config,2)+1;
                else
                    Settings.StatusDisplay.String = 'Invalid file';
                end
        end
    end
    
    %%Shows the end user the proper orientation required to calibrate
    %the robot's physical readings
    function Orientation_Callback(~,~)
        %Called by Settings.Cal.Orient on the CalSettings uipanel
        val = CalSettings.SelectedObject.UserData;
        if Pins.Set
            switch val
                case 1
                    if ~Pins.Pot.Min(val).Set
                        Config([-135 0 0 0 0])
                        Settings.StatusDisplay.String = 'Move the robot to the configuration shown on the left, then click Set';
                    else
                        Config([135 0 0 0 0]);
                        Settings.StatusDisplay.String = 'Now move the robot to the New configuration shown , then click Set';
                    end
                    view(-90,90)
                case 2
                    if ~Pins.Pot.Min(val).Set
                        Config([0 0 90 0 0]);
                        Settings.StatusDisplay.String = 'For this calibration, try to get the arm vertical with the base of the housing for M3 touching the top of the battery pack, then click Set';
                    else
                        Config([0 180 -90 0 0]);
                        Settings.StatusDisplay.String = 'For this Calibration, move M4 down until the joint Clicks, then click Set';
                    end
                    view(180,0)
                case 3
                    if ~Pins.Pot.Min(val).Set
                        Config([0 180 -150 0 0]);
                        Settings.StatusDisplay.String = 'For this calibration, turn M3 until it''s hitting the housing for M4, then click Set';
                    else
                        Config([0 0 150 0 0]);
                        Settings.StatusDisplay.String = 'For this Calibration, turn M3 until it''s hitting the housing for M4, then click Set';
                    end
                    view(180,0)
                case 4
                    if ~Pins.Pot.Min(val).Set
                        Config([0 90 0 -60 0]);
                        Settings.StatusDisplay.String = 'For this calibration, turn M2 until it clicks';
                    else
                        Config([0 90 0 60 0]);
                        Settings.StatusDisplay.String = 'For this Calibration, turn M2 until it clicks';
                    end
                    view(180,0)
                case 5
                    if ~Pins.Pot.Min(val).Set
                        Config([0 90 0 0 0]);
                        Settings.StatusDisplay.String = 'For this calibration, turn M1 until it clicks';
                    else
                        Config([0 90 0 0 45]);
                        Settings.StatusDisplay.String = 'For this Calibration, turn M1 until it clicks';
                    end
                    view(-90,0)
            end
            Settings.Cal.Set.Visible = 'on';
            Settings.Cal.Set.String = 'Set';
        else
            Status(Settings.StatusDisplay,'Please Setup Pins');
        end
    end
    
    %%Updates the plot with the physical robot's current orientation
    function PlotUpdate_Callback(~,~)
        %called by Data.UpdatePlot in the Main Figure
        if  CheckCal(Pins.Pot)
            Config(GetRobotConfig(Arduino,Pins,Data.ArmLimits,Data.Slide,Data.Radians));
        else
            Settings.StatusDisplay.String = 'Can''t Update plot yet';
            Status(Settings.StatusDisplay,'please calibrate pins');
        end
    end
    
    %%Changes the arm Movement power
    function PowerText_Callback(source,~)
        Data.PowerSlide(source.UserData).Value = str2double(source.String);        
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
                if  CheckCal(Pins.Pot)
                    Settings.StatusDisplay.String = 'Uploading';
                    for i = 1:saveindex-1
                        Config(Program.Config{i});
                        Upload_Callback(source);
                    end
                    Settings.StatusDisplay.String = 'Done';
                else
                    Status(Settings.StatusDisplay,'Calibrate pins first');
                    Status(Settings.StatusDisplay,'It is required');
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
            Program.Config{saveindex} = GetPlotConfig(Data.Slide,Data.Radians);
            Settings.StatusDisplay.String = Program.Config{saveindex};
            Status(Settings.StatusDisplay,strcat('Config?',num2str(saveindex),'?saved'));
            Program.List.String{saveindex} = strcat('Config?',num2str(saveindex));
            saveindex = saveindex + 1;
        elseif strcmp(source.Tag,'Delete')&& saveindex>2
            saveindex = saveindex -1;
            Config(Program.Config{saveindex-1});
            Program.List.String{saveindex} = [];
            Settings.StatusDisplay.String = strcat('Config?',num2str(saveindex),'?deleted');
        else
            saveindex = 1;
            Program.Config = {};
            Program.List.String = {};
            Program.List.Value = saveindex;
            Config([0 0 0 0 0]);
            Settings.StatusDisplay.String = 'Motion cancelled';
        end
    end
    
    %%Changes the Angles shown next to the sliders to degrees or radians
    function Radian_Callback(source,~)
        %Called by Data.Radians in the ViewTab
        for i = 1:4
            Data.SlideText(i).String = Data.Slide(i).Value.*RadCheck(i,Data.Slide,Data.Radians);
        end
        if source.Value
            source.String = 'Degrees';
        else
            source.String = 'Radians';
        end
    end
    
    %Informs the user what buttons move the selected motor
    function Remote_Callback(source,~)
        %called by Settings.Remote<> in the RemoteSettings uipanel
        if Pins.Set
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
        %Called by Settings.Remote<> in the RemoteSettings uipanel
        if Pins.Set
            if strcmp(event.Source.String,'M5')
                options = {'rightarrow','leftarrow'};
            elseif strcmp(event.Source.String,'M1')
                options = {'downarrow','uparrow'};
            else
                options = {'uparrow','downarrow'};
            end
            key = event.Key;
            val = event.Source.UserData;
            EmStop(Arduino,Active,Pins.En,val);
            if strcmp(key,options{1,1})
                move(val,'+');
            elseif strcmp(key,options{1,2})
                move(val,'-');
            else
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
                ResetArms_Callback(Data.SlideReset(i));
            end
        end
        Update(Data)
    end
    
    %%resets the axis limits
    function ResetAxis_Callback(~,~)
        %Called by Settings.Axis.Reset in the ViewTab
        axis([-350 350 -350 350  -75 625]);
%         axis([200 350 -75 75  -75 75]);
        Settings.Axis.Min(1).String = -350;
        Settings.Axis.Max(1).String = 350;
        Settings.Axis.Min(2).String = -350;
        Settings.Axis.Max(2).String = 350;
        Settings.Axis.Min(3).String = -75;
        Settings.Axis.Max(3).String = 625;
    end
    
    %%Resets the current slide limits or all slide limits
    function ResetLimits_Callback(source,~)
        %Called by Settings.Slide.Reset and Reset2 in the SlideLimits
        %uipanel
        switch source.UserData
            case 'single'
                val = Settings.Slide.Menu.Value;
                Data.Slide(val).Min = Data.ArmLimits(val,1);
                Data.Slide(val).Max = Data.ArmLimits(val,2);
                Settings.Slide.Min(val).String = Data.Slide(val).Min*RadCheck(val,Data.Slide,Data.Radians);
                Settings.Slide.Max(val).String = Data.Slide(val).Max*RadCheck(val,Data.Slide,Data.Radians);
                Data.Slide(val).Value = 0;
                Data.SlideText(val).String = 0;
            case 'all'
                for i = 1:5
                    Data.Slide(i).Min = Data.ArmLimits(i,1);
                    Data.Slide(i).Max = Data.ArmLimits(i,2);
                    Settings.Slide.Min(i).String = Data.Slide(i).Min*RadCheck(i,Data.Slide,Data.Radians);
                    Settings.Slide.Max(i).String = Data.Slide(i).Max*RadCheck(i,Data.Slide,Data.Radians);
                end
                ResetArms_Callback(Data.ResetArm);
        end
    end
    
    %%Saves data to disk
    function Save_Callback(source,~)
        switch source.Parent.Title
            case 'Pin Settings'
                if Pins.Set
                    File = [];
                    File{1} = Pins;
                    File{2} = str2double(Settings.Pin.Light.String);
                    File{3} = str2double(Settings.Pin.Interrupt.String);
                    File{4} = Active;
                    uisave('File','Settings/Pins/Pins.mat')
                    File = [];
                else
                    Settings.StatusDisplay.String = 'Please set pins first';
                end
            case 'Calibration'
                if CheckCal(Pins.Pot)
                    File = {Pins.Pot(:)};
                    uisave('File','Settings/Cal/Calibration.mat')
                    File = [];
                else
                    Status(Settings.StatusDisplay,'please calibrate pins first');
                end
            case 'Open Loop Automation'
            case 'Closed Loop Automation'
                if numel(Program.Config)
                    File = Program.Config;
                    uisave('File','Settings/Move/Movement')
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
            Apin = {Settings.Pin.APin.String Settings.Pin.Interrupt.String};
            En = {Settings.Pin.En.String Settings.Pin.Light.String};
            Dir = {Settings.Pin.Dir.String};
            Display = Settings.StatusDisplay;
            if  ImproperPinValues(Display,Apin,En,Dir)
                Status(Display,sprintf('Recheck Values'))
            else
                Status(Display,sprintf('Values Check Out'))
                for i = 1:5
                    Pins.Ana(i) = eval(Apin{i});
                    Pins.En(i) = eval(En{i});
                    Pins.Dir(i) = eval(Dir{i});
                end
                Active = Settings.Pin.High.Value;
                Pins.Set = 1;
                Pins.Light = str2double(Settings.Pin.Light.String);
                Pins.Interrupt = str2double(Settings.Pin.Interrupt.String);
                IRQ = strcat('A',Settings.Pin.Interrupt.String);
                configurePin(Arduino,IRQ,'Pullup');
                CalSettings.SelectedObject = Settings.Cal.Button(1);
                CalButton_Callback(Settings.Cal.Button(1));
                CalPanel.Visible = 'on';
                Stop;
            end
        else
            Settings.StatusDisplay.String = 'Arduino board not passed to GUI';
        end
    end
    
    %%Changes the configuration of the robot arm
    function Slide_Callback(source,~)
        %Called by Data.Slide<> in the ViewTab
        index = source.UserData;
        if strcmp(source.String,'P')
            Data.PowerText(index).String = num2str(source.Value,4);
        else
            Data.SlideText(index).String = num2str(source.Value*RadCheck(index,Data.Slide,Data.Radians),3);
            Update(Data)
        end
    end
    
    %%Sets the limit of the chosen slide
    function SlideLimit_Callback(source,~)
        %Called by Settings.Slide.Label in the SlideSettings uipanel
        val = source.UserData;
        if ~isempty(Settings.Slide.Min(val).String)&&~isempty(Settings.Slide.Max(val).String)
            Data.Slide(val).Min = str2double(Settings.Slide.Min(val).String)/RadCheck(val,Data.Slide,Data.Radians);
            Data.Slide(val).Max = str2double(Settings.Slide.Max(val).String)/RadCheck(val,Data.Slide,Data.Radians);
            Data.Slide(val).Value = str2double(Settings.Slide.Max(val).String)/RadCheck(val,Data.Slide,Data.Radians);
            Data.SlideText(val).String = Data.Slide(val).Value*RadCheck(val,Data.Slide,Data.Radians);
            Update(Data) 
        end
    end
    
    %%Sets slide values
    function SlideText_Callback(source,~)
        %Called by Data.SlideText<> in the ViewTab
        index = source.UserData;
        Data.Slide(index).Value = str2double(source.String)/RadCheck(index,Data.Slide,Data.Radians);
        Update(Data)
    end
    
    %EMERGENCY STOP
    function Stop(~,~)
        Interrupt = 1;
        if Pins.Set
            digitalWrite(Arduino,Pins.En,zeros(size(Pins.En)));
        else
            Status(Settings.StatusDisplay,'-_- Stop what?');
        end
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
                RemoteSettings.Position = [175/xmax2 22/ymax2 26/xmax2 120/ymax2];
                Settings.StatusDisplay.Parent = ArduinoTab;
                Settings.StatusDisplay.String = '';
                Data.Stop.Position = [662.5/750,125/500,70/750,80/500];
            case 'Display'
                Settings.StatusDisplay.Parent = ViewTab;
                RemoteSettings.Parent = ViewTab;
                RemoteSettings.Position = [2/xmax1 345/ymax1 26/xmax1 152/ymax1];
                Settings.StatusDisplay.String = '';
                if Settings.Advanced1.Value || Settings.Advanced2.Value || Settings.DH.Value
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
        goal = [Data.Slide(val).Value];
        if CheckCal(Pins.Pot)
            move(val,goal);
        elseif ~Pins.Set
            Status(Settings.StatusDisplay,'Setup pins first');
        else
            Status(Settings.StatusDisplay,'Pins need to be calibrated');
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
            Settings.Slide.Min(val).Visible = 'on';
            Settings.Slide.Max(val).Visible = 'on';
            Settings.Slide.Label(val).Visible = 'on';
            Settings.Slide.Menu.Enable = 'off';
        else
            Settings.Slide.TopLabel.Visible = 'off';
            Settings.Slide.Min(val).Visible = 'off';
            Settings.Slide.Max(val).Visible = 'off';
            Settings.Slide.Label(val).Visible = 'off';
            Settings.Slide.Menu.Enable = 'on';
        end
    end

%% FUNCTIONS

%SET AN ARM CONFIGURATION (angle values in degrees)
    function Config(input,varargin)
        if nargin == 1
            for i = 1:numel(input)
                Data.Slide(i).Value = input(i)/RadCheck(i,Data.Slide,Data.Radians);
                Slide_Callback(Data.Slide(i));
            end
        else
            SlideConfig = GetPlotConfig(Data.Slide,Data.Radians);
            for i = 1:5
                n = input(6-i) - SlideConfig(6-i);
                m = n - rem(n,1);
                if m
                    for k = 1:abs(m)
                        Data.Slide(6-i).Value = (SlideConfig(6-i)+k*m/abs(m))/RadCheck(6-i,Data.Slide,Data.Radians);
                        pause(.01);
                    end
                    if rem(n,1)
                        Data.Slide(6-i).Value = input(6-i)/RadCheck(6-i,Data.Slide,Data.Radians);
                    end
                end
                Slide_Callback(Data.Slide(6-i));
            end
        end
    end
    
%GET DIRECTION TOWARDS GOAL
    function [Direction,Dist] = getDir(val,goal)
        Direction = ones(size(val));
        if isnumeric(goal)            
            range = [Pins.Pot.Max(val).Value] - [Pins.Pot.Min(val).Value];
            scale = range./abs(range); %([Pins.Pot.Max(val).Value] - [Pins.Pot.Min(val).Value])./abs([Pins.Pot.Max(val).Value] - [Pins.Pot.Min(val).Value]); 
            goalVoltage = map(goal',Data.ArmLimits(val,:),[[Pins.Pot.Min(val).Value]' [Pins.Pot.Max(val).Value]'].*scale');  
            goalOffset = (goalVoltage - analogRead(Arduino,(Pins.Ana(val))').*scale');
            Dist = abs(goalOffset);
            Direction(abs(goalOffset) < tolerance) = NaN;
            Direction(abs(goalOffset) > tolerance & goalOffset < 0) = 0;
            Direction(abs(goalOffset) > tolerance & goalOffset > 0) = 1;
        else
            Dist = Direction;
            Direction(strcmp(goal,'-')) = 0;
            Direction(strcmp(goal,'+')) = 1;
            Direction(~(strcmp(goal,'+')|strcmp(goal,'-'))) = NaN;
        end
    end
    
    %MOVE ARM
    function move(val,goal)
        clc
        Dir = getDir(val,goal);
        IdlePins = isnan(Dir);
        digitalWrite(Arduino,Pins.Dir(val(~IdlePins)),Dir(~IdlePins));
        Interrupt = 0;
        if analogRead(Arduino,Pins.Interrupt)> 4
            if ~(strcmp(goal,'+')||strcmp(goal,'-')) && sum(~IdlePins)
                %analogWrite(Arduino,Pins.En(val(~IdlePins)),Data.PowerSlide(Val(~IdlePins)).Value/100*2.5*ones(numel(find(~IdlePins)),1));
                digitalWrite(Arduino,Pins.En(val(~IdlePins)),ones(numel(find(~IdlePins)),1));
                n = numel(find(IdlePins));
                while (n < numel(Dir) && ~Interrupt && analogRead(Arduino,Pins.Interrupt) > 4)
                    Dir = getDir(val,goal);
                    NewIdlePins = isnan(Dir);
                    n = numel(find(NewIdlePins));
                    NewIdlePins = xor(NewIdlePins,IdlePins);
                    pause(0.1);
                    if sum(NewIdlePins)
                        digitalWrite(Arduino,Pins.En(val(NewIdlePins)),zeros(numel(find(NewIdlePins)),1));
                    end
                    IdlePins = IdlePins|NewIdlePins;
                end
                EmStop(Arduino,Active,Pins.En,(1:5));
            else
                for i = 1:numel(val)
                    if strcmp(goal(i),'+')||strcmp(goal(i),'-')
                        analogWrite(Arduino,Pins.En(val(i)),Data.PowerSlide(val(i)).Value/100 * 2.5);
                        Pins.En(val(i))
                        Data.PowerSlide(val(i)).Value/100 * 2.5
                    else
                        EmStop(Arduino,Active,Pins.En,val(i));
                    end
                end
            end
        else
            Status(Settings.StatusDisplay,'Remove Remote Jumper/ release STOP switch');
        end
    end
    
end
