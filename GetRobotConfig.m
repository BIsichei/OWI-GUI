function result = GetRobotConfig(Arduino,Pins,Limits,Slides,Rad)
    radresult = map(analogRead(Arduino,Pins.Ana'),[[Pins.Pot.Min.Value]' [Pins.Pot.Max.Value]'],Limits);
    result = radresult.*RadCheck(1:5,Slides,Rad)';
end
