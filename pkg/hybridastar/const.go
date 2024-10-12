package hybridastar

import "math"

const M_PI float32 = 3.14159265358979323846

// _________________
// CONFIG FLAGS

// A flag for additional debugging output via `std::cout`
const coutDEBUG bool = false

// A flag for the mode (true = manual; false = dynamic). Manual for  map or dynamic for dynamic map.
const manual bool = true

// A flag for the visualization of 3D nodes (true = on; false = off)
const visualization bool = false && manual

// A flag for the visualization of 2D nodes (true = on; false = off)
const visualization2D bool = false && manual

// A flag to toggle reversing (true = on; false = off)
const reverse bool = true

// A flag to toggle the connection of the path via Dubin's shot (true = on; false = off)
const dubinsShot bool = true

// A flag to toggle the Dubin's heuristic, this should be false, if reversing is enabled (true = on; false = off)
const dubins bool = false

/*
!

	\var  const bool dubinsLookup
	\brief A flag to toggle the Dubin's heuristic via lookup, potentially speeding up the search by a lot
	\todo not yet functional
*/
const dubinsLookup bool = false && dubins

// A flag to toggle the 2D heuristic (true = on; false = off)
const twoD bool = true

// _________________
// GENERAL CONSTANTS

// [#] --- Limits the maximum search depth of the algorithm, possibly terminating without the solution
const iterations int32 = 30000

// [m] --- Uniformly adds a padding around the vehicle
const bloating float64 = 0

// [m] --- The width of the vehicle
const width float64 = 1.75 + 2*bloating

// [m] --- The length of the vehicle
const length float64 = 2.65 + 2*bloating

// [m] --- The minimum turning radius of the vehicle
const r float32 = 6

// [m] --- The number of discretizations in heading
const headings int32 = 72

// [°] --- The discretization value of the heading (goal condition)
const deltaHeadingDeg float32 = 360 / float32(headings)

// [c*M_PI] --- The discretization value of heading (goal condition)
const deltaHeadingRad float32 = 2 * M_PI / float32(headings)

// [c*M_PI] --- The heading part of the goal condition
const deltaHeadingNegRad float32 = 2*M_PI - deltaHeadingRad

// [m] --- The cell size of the 2D grid of the world
const cellSize float32 = 1

/*
!

	\brief [m] --- The tie breaker breaks ties between nodes expanded in the same cell

	As the cost-so-far are bigger than the cost-to-come it is reasonbale to believe that the algorithm would prefer the predecessor rather than the successor.
	This would lead to the fact that the successor would never be placed and the the one cell could only expand one node. The tieBreaker artificially increases the cost of the predecessor
	to allow the successor being placed in the same cell.
*/
const tieBreaker float32 = 0.01

// ___________________
// HEURISTIC CONSTANTS

// [#] --- A factor to ensure admissibility of the holonomic with obstacles heuristic
var factor2D float32 = float32(math.Sqrt(5)/math.Sqrt(2)) + 1

// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
const penaltyTurning float32 = 1.05

// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
const penaltyReversing float32 = 2.0

// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
const penaltyCOD float32 = 2.0

// [m] --- The distance to the goal when the analytical solution (Dubin's shot) first triggers
const dubinsShotDistance float32 = 100

// [m] --- The step size for the analytical solution (Dubin's shot) primarily relevant for collision checking
const dubinsStepSize float32 = 1

// ______________________
// DUBINS LOOKUP SPECIFIC

// [m] --- The width of the dubinsArea / 2 for the analytical solution (Dubin's shot)
const dubinsWidth int32 = 15

// [m] --- The area of the lookup for the analytical solution (Dubin's shot)
const dubinsArea int32 = dubinsWidth * dubinsWidth

// _________________________
// COLLISION LOOKUP SPECIFIC

// [m] -- The bounding box size length and width to precompute all possible headings
var bbSize int32 = int32(math.Ceil((math.Sqrt(width*width+length*length) + 4) / float64(cellSize)))

// [#] --- The sqrt of the number of discrete positions per cell
const positionResolution int32 = 10

// [#] --- The number of discrete positions per cell
const positions int32 = positionResolution * positionResolution

// A structure describing the relative position of the occupied cell based on the center of the vehicle
type relPos struct {
	/// the x position relative to the center
	x int32
	/// the y position relative to the center
	y int32
}

// A structure capturing the lookup for each theta configuration
type config struct {
	/// the number of cells occupied by this configuration of the vehicle
	length int32
	/*!
	  \var relPos pos[64]
	  \brief The maximum number of occupied cells
	  \todo needs to be dynamic
	*/
	pos [64]relPos
}

// _________________
// SMOOTHER SPECIFIC
// [m] --- The minimum width of a safe road for the vehicle at hand
const minRoadWidth float32 = 2

// ____________________________________________
// COLOR DEFINITIONS FOR VISUALIZATION PURPOSES
// / A structure to express colors in RGB values
type color struct {
	/// the red portion of the color
	red float32
	/// the green portion of the color
	green float32
	/// the blue portion of the color
	blue float32
}

// A definition for a color used for visualization
var teal = color{102 / 255, 217 / 255, 239 / 255}

// A definition for a color used for visualization
var green = color{166 / 255, 226 / 255, 46 / 255}

// // A definition for a color used for visualization
var orange = color{253 / 255, 151 / 255, 31 / 255}

// // A definition for a color used for visualization
var pink = color{249 / 255, 38 / 255, 114 / 255}

// // A definition for a color used for visualization
var purple = color{174 / 255, 129 / 255, 255 / 255}
