function result = EmptyEntries(Array)
    result = 0;
    for i = 1:numel(Array)
        value = str2double(Array(i).String);
        if isempty(Array(i).String)
            result = 1;
            break;
        elseif isnan(value)||mod(value,1)~=0
            result = 2;
            break;
        end
    end
end
