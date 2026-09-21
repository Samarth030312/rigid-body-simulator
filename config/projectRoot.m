function root = projectRoot()
% projectRoot Return the absolute project root.
thisFile = mfilename('fullpath');
root = fileparts(fileparts(thisFile));
end
