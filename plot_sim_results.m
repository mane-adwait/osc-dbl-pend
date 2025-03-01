% Plot the results from the simulation.
% Adwait Mane, 2025 February 25.

load("data.mat")

%% Generate plots.

% Create the figure and set the properties:
fig5 = figure(5); % For convenience, use figN = figure(N)
cFig = gcf; % This makes the following commands modular i.e. the figure 
% handle does not need to be updated.
% cFig.Name = [fileName, fileVersion]; 

% Set the figure position using the syntax [left bottom width height].
cFig.Units = 'normalized'; cFig.OuterPosition = [0.5 0.5 0.5 0.5];
% Another way to specify figure location. Acceptable arguments include:
% Eight compass directions: 'north', 'northeast', 'east', etc.
% 'center' – centers the figure in the middle of the screen.
% 'onscreen' – moves the figure fully onto the screen if it's partially off-screen.
% movegui(cFig,'northeast'); 

cFig.Color = 'white'; % I think the 'no background' option appears black in
% the Matlab environment, but not when exported. The white background can be
% removed in Inkscape.