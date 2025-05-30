function imuMeasLeft = importIMUMeasFromGithub(filename, dataLines)
    %IMPORTFILE Import data from a text file
    %  IMUMEASLEFT = IMPORTFILE(FILENAME) reads data from text file FILENAME
    %  for the default selection.  Returns the numeric data.
    %
    %  IMUMEASLEFT = IMPORTFILE(FILE, DATALINES) reads data for the
    %  specified row interval(s) of text file FILENAME. Specify DATALINES as
    %  a positive scalar integer or a N-by-2 array of positive scalar
    %  integers for dis-contiguous row intervals.
    %
    %  Example:
    %  imuMeasLeft = importfile("G:\My Drive\Study\Gait_data03\gait0706\test06\imuMeasLeft.csv", [2, Inf]);
    %
    %  See also READTABLE.
    %
    % Auto-generated by MATLAB on 03-Feb-2025 16:55:01

    %% Input handling

    % If dataLines is not specified, define defaults
    if nargin < 2
        dataLines = [2, Inf];
    end

    %% Set up the Import Options and import the data
    opts = delimitedTextImportOptions("NumVariables", 8);

    % Specify range and delimiter
    opts.DataLines = dataLines;
    opts.Delimiter = ",";

    % Specify column names and types
    opts.VariableNames = ["time_index", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z", "is_zero_vel"];
    opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double"];

    % Specify file level properties
    opts.ExtraColumnsRule = "ignore";
    opts.EmptyLineRule = "read";

    % Import the data
    imuMeasLeft = readtable(filename, opts);

    %% Convert to output type
    imuMeasLeft = table2array(imuMeasLeft);
end
