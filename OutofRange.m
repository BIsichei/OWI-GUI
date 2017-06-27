function result = OutofRange(Pins,Type)
    result = 0;
    switch Type
        case 'D' %Digital
            Limits = [2 54];
        case 'A' %Analog
            Limits = [0 16];
    end
    for i = 1:numel(Pins)
        if Pins(i)>Limits(2) || Pins(i)<Limits(1)
            result = 1;
            break
        end
    end
end
