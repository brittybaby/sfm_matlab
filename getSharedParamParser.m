% Return shared inputParser
function parser = getSharedParamParser(filename)

% Parse the PV-pairs
defaults = struct('MarkerSize', 6, 'VerticalAxis', 'Z', ...
    'VerticalAxisDir', 'Up');

% Setup parser
parser = inputParser;
parser.CaseSensitive = false;
parser.FunctionName  = filename;

parser.addParameter('MarkerSize', defaults.MarkerSize, ...
    @(x)validateMarkerSize(filename, x));

parser.addParameter('VerticalAxis', defaults.VerticalAxis, ...
    @(x)validateVerticalAxis(filename, x));

parser.addParameter('VerticalAxisDir', defaults.VerticalAxisDir, ...
    @(x)validateVerticalAxisDir(filename, x));

  