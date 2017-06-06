function result = RadCheck(index,Slides,Rad)
    %CONVERT FROM RAD TO DEGRESS IF RAD BUTTON IS PUSHED
    result = 1;
    if strcmp(Slides(index).String,'R') && ~Rad.Value
        result = 180/pi;
    end
end
