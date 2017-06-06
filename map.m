function result = map(val,angleLims,voltLims)
    %MAP FUNCTION
    x1 = angleLims(:,1);
    y1 = angleLims(:,2);
    x2 = voltLims(:,1);
    y2 = voltLims(:,2);
    result = (val - x1').*(y2' - x2')./(y1'-x1')+ x2';
end
