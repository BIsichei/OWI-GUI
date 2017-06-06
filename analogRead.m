function result = analogRead(Arduino,pin)
    %ANALOG READ
    pin = strcat('A',num2str(pin));
    result = zeros(size(pin));
    for i = 1:size(pin,1)
        result(i) = Arduino.readVoltage(pin(i,:));
    end
end
