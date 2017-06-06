function result = CheckCal(SetArray)
    % CHECK IF ALL PINS HAVE BEEN CALIBRATED    
    result = 1;
    for val = 1:5
        if ~(strcmp(SetArray(val,1),'Set')&&strcmp(SetArray(val,2),'Set'))
            result = 0;
            break
        end
    end
end
