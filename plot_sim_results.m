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


% Create the axes

% -------------------------------------------------------------------------
% ax1_fig5 = axes; % Create a single axes object for the figure.

% subplot(m,n,p) creates an m-by-n grid. p=1 is first column of the first
% row. p=2 is the second column of the first row, and so on.
%   1   2
%   3   4
%   5   6 ...

% -------------------------------------------------------------------------

draw.m = 2; draw.n = 1; % Select the subplot grid size.

%% Set the axes properties.

for im = 1:draw.m
    for in = 1:draw.n

        numPlots = 2; % Select the number of plots on each subplot.

        subplot(draw.m,draw.n,im); % Select the subplot axes.
        % Note: update the last argument if plotting more than one column.

        % Set the axes properties
        cAx = gca;
        lineColors = linspecer(numPlots);
        axis on; cAx.TickLabelInterpreter = 'latex';
        cAx.ColorOrder = lineColors; cAx.FontSize = 20;
        hold on;

    end
end

%% Plot the states on subplots.

subplot(draw.m,draw.n,1); 