package hybridastar

import "github.com/haormj/path-planner/pkg/pb"

type CollisionDetection struct {
	grid *pb.OccupancyGrid
}

func NewCollisionDetection() {
	//		this->grid = nullptr;
	//	  Lookup::collisionLookup(collisionLookup);
}

/*
!

	\brief evaluates whether the configuration is safe
	\return true if it is traversable, else false
*/
func (d *CollisionDetection) IsTraversable(node Node) bool {
	/* Depending on the used collision checking mechanism this needs to be adjusted
	   standard: collision checking using the spatial occupancy enumeration
	   other: collision checking using the 2d costmap and the navigation stack
	*/
	var cost float32 = 0

	// 2D collision test
	if node.GetT() == 99 {
		return d.grid.Data[int(node.GetIdx())] != 0
	}

	if true {
		if !d.configurationTest(float32(node.GetX()), float32(node.GetY()), node.GetT()) {
			cost = 1
		}
	} else {
		cost = d.configurationCost(float32(node.GetX()), float32(node.GetY()), node.GetT())
	}

	return cost <= 0
}

/*
!

	\brief Calculates the cost of the robot taking a specific configuration q int the World W
	\param x the x position
	\param y the y position
	\param t the theta angle
	\return the cost of the configuration q of W(q)
	\todo needs to be implemented correctly
*/
func (d *CollisionDetection) configurationCost(x, y, t float32) float32 { return 0 }

func (d *CollisionDetection) configurationTest(x, y, t float32) bool {
	// X := int32(x);
	// Y := int32(y);
	// int iX = (int)((x - (long)x) * positionResolution);
	// iX = iX > 0 ? iX : 0;
	// int iY = (int)((y - (long)y) * positionResolution);
	// iY = iY > 0 ? iY : 0;
	// int iT = (int)(t / Constants::deltaHeadingRad);
	// int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
	// int cX;
	// int cY;

	// for (int i = 0; i < collisionLookup[idx].length; ++i) {
	//   cX = (X + collisionLookup[idx].pos[i].x);
	//   cY = (Y + collisionLookup[idx].pos[i].y);

	//   // make sure the configuration coordinates are actually on the grid
	//   if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
	// 	if (grid->data[cY * grid->info.width + cX]) {
	// 	  return false;
	// 	}
	//   }
	// }

	return true
}
