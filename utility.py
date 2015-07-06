class Grid:
    def __init__(self, grid_coord, grid_count):
        self.grid_coord = grid_coord
        self.grid_count = grid_count
        self.state = '-'
        self.ID = None

    def setID(self, ID):
        self.ID = ID

    def pixelToGridCoord(self, coord):
        pass

    def isOccupied(self):
        return self.state == 'o'

    def gridCoord(self):
        return self.grid_coord

    def occupy(self):
        self.state = 'o'

    def complement(self):
        self.state = 'c'

    def isPrintable(self):
        return self.state != 'c'

    def surroundingGrids(self):
        x, y = self.grid_coord
        # diagonal grids not included
        temp_coords = ((x, y - 1), (x - 1, y), (x + 1, y), (x, y + 1))
        for coord in temp_coords:
            if coord[0] >= 0 and coord[0] < self.grid_count:
                if coord[1] >= 0 and coord[1] < self.grid_count:
                    yield coord
    
class GridMap:
    def __init__(self, grid_count, grid_size):
        self.grid_count = grid_count
        self.grid_size = grid_size
        self.grids = self.createGrids(grid_count)

    def createGrids(self, size):
        return [[Grid((i, j), self.grid_count) for i in range(size)] for j in range(size)]

    def grid(self, x, y):
        return self.grids[y][x]

    def allGrids(self):
        #return [grid for grid in [row for row in self.grids]]
        for row in self.grids:
            for grid in row:
                yield grid

    def occupiedGrids(self):
        coords = []
        for row in self.grids:
            for grid in row:
                if grid.isOccupied():
                    coords.append(grid.gridCoord())
        return coords

    def pixelToGridCoord(self, coord):
        return coord[0] / self.grid_size, coord[1] / self.grid_size

    def clicked(self, coord):
        grid_coord = self.pixelToGridCoord(coord)
        self.grid(*grid_coord).occupy()

    def __str__(self):
        return '\n'.join([str([grid.state for grid in self.grids[i]]) for i in range(self.grid_count)])

class Surface:
    def __init__(self, grid_map):
        self.grid_map = grid_map
        self.complement_grids = self.getComplementGrid()
        self.is_closed = self.isClosedSurface()
        self.is_vertically_convexed = self.isVerticallyConvexedSurface()
        self.is_horizontally_convexed = self.isHorizontallyConvexedSurface()

    def isValidShape(self):
        return self.is_vertically_convexed or self.is_horizontally_convexed

    def isClosedSurface(self):
        total_grid_count = self.grid_map.grid_count**2
        # assuming that a closed surface must enclose at least one grid
        # within boundary 
        return total_grid_count > \
               len(self.complement_grids) + len(self.grid_map.occupiedGrids())

    def isVerticallyConvexedSurface(self):
        for grid in self.complement_grids:
            x, y = grid
            top_clear = True
            for y_coord in xrange(y - 1, -1, -1):
                if self.grid_map.grid(x, y_coord).isOccupied():
                    top_clear = False
                    break
            for y_coord in xrange(y + 1, self.grid_map.grid_count):
                if self.grid_map.grid(x, y_coord).isOccupied():
                    if not top_clear: return False

        return True         

    def isHorizontallyConvexedSurface(self):
        for grid in self.complement_grids:
            x, y = grid
            left_clear = True
            for x_coord in xrange(x - 1, -1, -1):
                if self.grid_map.grid(x_coord, y).isOccupied():
                    left_clear = False
                    break
            for x_coord in xrange(x + 1, self.grid_map.grid_count):
                if self.grid_map.grid(x_coord, y).isOccupied():
                    if not left_clear: return False

        return True         

    def getComplementGrid(self):
        complement = []
        
        def setComplementGrid(coord, complement):
            complement.append(coord)
            for grid_coord in self.grid_map.grid(*coord).surroundingGrids():
                if grid_coord not in complement:
                    if not self.grid_map.grid(*grid_coord).isOccupied():
                        setComplementGrid(grid_coord, complement)
                        
        # assuming that first coord will always be non occupied
        setComplementGrid((0, 0), complement)
        return complement

    def fillSurface(self):
        for grid_coord in self.complement_grids:
            self.grid_map.grid(*grid_coord).complement()
            
###### test ####
##a= GridMap(5,5)
##a.grid(1, 1).occupy()
##a.grid(1, 2).occupy()
##a.grid(2, 1).occupy()
##a.grid(2, 2).occupy()
##b = Surface(a)
##
##
##print a
##print "len(b.complement_grids):", len(b.complement_grids)
##print "len(a.occupiedGrids()):", len(a.occupiedGrids())
##print "b.isVerticallyConvexedSurface():", b.isVerticallyConvexedSurface()
##print "b.isHorizontallyConvexedSurface():", b.isHorizontallyConvexedSurface()
##print "b.isClosedSurface():", b.isClosedSurface()
##print "b.isValidShape():", b.isValidShape()
###### test ####
