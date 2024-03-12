% Specify the file path
file_path = '/Data/ActualPos.txt';

% Read the file using fopen and textscan
fid = fopen(file_path, 'r');
file_data = fileopen(fid, '%s', 'Delimiter', '\n');
fclose(fid);

% Extract numeric values using regular expressions
parsed_data = [];
for i = 1:numel(file_data{1})
    line = file_data{1}{i};
    matches = regexp(line, '\[([^\[\]]+)\]', 'tokens');
    if ~isempty(matches)
        numbers = str2double(strsplit(matches{1}{1}, ','));
        parsed_data = [parsed_data; numbers];
    end
end

% Now parsed_data contains the numeric values from the file, each row representing a set of numbers from a line in the file.
