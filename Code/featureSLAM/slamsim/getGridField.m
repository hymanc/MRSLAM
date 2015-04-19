function field = getGridField(nrow,ncol)

field.INNER_OFFSET_X = 32;
field.INNER_OFFSET_Y = 13;

field.INNER_SIZE_X = 1000;
field.INNER_SIZE_Y = 400;

field.COMPLETE_SIZE_X = field.INNER_SIZE_X + 2 * field.INNER_OFFSET_X;
field.COMPLETE_SIZE_Y = field.INNER_SIZE_Y + 2 * field.INNER_OFFSET_Y;

field.MARKER_OFFSET_X = 21;
field.MARKER_OFFSET_Y = 0;

field.MARKER_DIST_X = 800;
field.MARKER_DIST_Y = 350;

row = field.MARKER_OFFSET_X + linspace(0,field.MARKER_DIST_X,ncol);
col_Y = field.MARKER_OFFSET_Y + linspace(0,field.MARKER_DIST_Y,nrow);

field.MARKER_X_POS = [];
field.MARKER_Y_POS = [];
for i = 1:nrow
   field.MARKER_X_POS = [field.MARKER_X_POS, row];
   field.MARKER_Y_POS = [field.MARKER_Y_POS, col_Y(i)*ones(size(row))];
end


field.NUM_MARKERS = length(field.MARKER_X_POS);
