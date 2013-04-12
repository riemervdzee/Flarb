#include "flarb_mapbuilder/MapImage.h"


class cMovement
{
public:
	int Create();
	int trendLine(flarb_mapbuilder::MapImage msg);
	int saveZone(float meterX, float meterY, const flarb_mapbuilder::MapImage &msg);
	int getPixel(int x , int y, const flarb_mapbuilder::MapImage &msg);
private:
	flarb_mapbuilder::MapImage msgr;
};
