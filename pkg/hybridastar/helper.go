package hybridastar

/*
!

	\fn float normalizeHeadingRad(float t)
	\brief Normalizes a heading given in rad to (0,2PI]
	\param t heading in rad
*/
func normalizeHeadingRad(t float32) float32 {
	if t < 0 {
		t = t - 2*M_PI*float32(int32(t/(2*M_PI)))
		return 2*M_PI + t
	}

	return t - 2*M_PI*float32((int32)(t/(2*M_PI)))
}
