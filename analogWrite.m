function analogWrite(Arduino,pin,Value)
    %ANALOG WRITE
    for i = 1:size(pin,1)
        Arduino.writePWMVoltage(strcat('D',num2str(pin(i))),Value(i));
    end
end
