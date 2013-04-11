#include "flarb_mapbuilder/MapImage.h"


class cMovement
{
public:
	int Create();
	int trendLine(flarb_mapbuilder::MapImage msg);
	int saveZone(float meterX, float meterY, flarb_mapbuilder::MapImage msg);
	int getPixel(int x , int y, flarb_mapbuilder::MapImage msg);
private:
	flarb_mapbuilder::MapImage msgr;
};
