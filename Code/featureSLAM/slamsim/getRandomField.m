function field = getRandomField( nmarks )
%GETRANDOMFIELD Summary of this function goes here
%   Detailed explanation goes here

field.INNER_OFFSET_X = 32;
field.INNER_OFFSET_Y = 13;

field.INNER_SIZE_X = 420;
field.INNER_SIZE_Y = 270;

field.COMPLETE_SIZE_X = field.INNER_SIZE_X + 2 * field.INNER_OFFSET_X;
field.COMPLETE_SIZE_Y = field.INNER_SIZE_Y + 2 * field.INNER_OFFSET_Y;

field.MARKER_OFFSET_X = 21;
field.MARKER_OFFSET_Y = 0;

field.MARKER_DIST_X = 442;
field.MARKER_DIST_Y = 292;

field.MARKER_X_POS = field.MARKER_OFFSET_X+(1.5*field.MARKER_DIST_X).*(rand(1,nmarks));
field.MARKER_Y_POS = field.MARKER_OFFSET_Y+(1.5*field.MARKER_DIST_Y).*(rand(1,nmarks));

field.NUM_MARKERS = length(field.MARKER_X_POS);
