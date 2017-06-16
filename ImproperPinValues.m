function result = ImproperPinValues(Display,Apin,Neg,Pos)
    % CHECK FOR LEGAL PIN VALUES
    result = 1;    
    % check for empty and non integer entries
    A = EmptyEntries(Apin);
    N = EmptyEntries(Neg);
    P = EmptyEntries(Pos);
    if A == 1 || N == 1  || P == 1 
        Status(Display,'Please input all Values');
    elseif A == 2 || N == 2  || P == 2 
        Status(Display,'Only integers allowed');
    else
        Pins = zeros(numel(Apin),3);
        for i = 1:numel(Apin)
            Pins(i,:) = [eval(Apin{i}) eval(Neg{i}) eval(Pos{i})];
        end
        % make sure pins are within the right range
        if OutofRange(Pins)
            error = OutofRange;
            if error == 1
                Status(Display,'An analog input pin is out of range');
            elseif error == 2
                Status(Display,'A negative output pin is out of range');
            else
                Status(Display,'A positive output pin is out of range');
            end
        % make sure pins are not repeated
        elseif RepeatedEntries(Pins(:,1))
            Status(Display,'Repeated analog input pin');
        elseif RepeatedEntries(Pins(:,2:3))
            Status(Display,'Repeated digital output pin');
        else
            result = 0;
        end
    end
    %{
    EmptyEntries
    Status
    OutofRange
    RepeatedEntries
    %}
end
