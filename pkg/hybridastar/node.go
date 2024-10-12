package hybridastar

type Node interface {
	GetX() int32
	GetY() int32
	GetT() float32
	GetIdx() int32
}
