#include "flarb_mapbuilder/MapImage.h"


class cMovement
{
public:
	int Create();
	int trendLine(flarb_mapbuilder::MapImage msg);
	int saveZone(float meterX, float meterY, const flarb_mapbuilder::MapImage &msg);

	int CheckBlockedPixel( const flarb_mapbuilder::MapImage &msg, int x, int y);
	int CheckBlockedLineX( const flarb_mapbuilder::MapImage &msg, int x, int y, int width);
	int CheckBlockedLineY( const flarb_mapbuilder::MapImage &msg, int x, int y, int height);
private:
	flarb_mapbuilder::MapImage msgr;
};
