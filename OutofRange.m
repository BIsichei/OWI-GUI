function result = OutofRange(Pins)
    result = 0;
    for i = 1:5
        pos = Pins(i,3);
        neg = Pins(i,2);
        apin = Pins(i,1);
        if apin>16 || apin< 0
            result = 1;
        end
        if neg>54 || neg< 2
            result = 2;
        end
        if pos>54 || pos< 2
            result = 3;
        end
        if result
            break;
        end
    end    
end
