#ifndef CLASS_MAP_H
#define CLASS_MAP_H

#include <vector>
#include "flarb_simulation/types/tVector.h"

struct Plant {
	float x;
	float y;
};

class cMap {
public:
	// Functions executed at the begining and end of the Application
	bool Create();
	void Destroy();

	// Draws the map via Graphics extension
	void Draw() const;

	//
	float TestRayDistance( tVector l1, float angle) const;

	//
	bool IntersectCircle( tVector l1, tVector l2, tVector circle,
		float radius, float &result) const;

private:
	std::vector<Plant> _plants;
};

#endif // CLASS_MAP_H
