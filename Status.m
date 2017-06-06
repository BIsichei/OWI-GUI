function Status(Display,string)
    %APPEND STATUS MESSAGE TO STATUS DISPLAY    
    Display.String = [Display.String;{string}];
end