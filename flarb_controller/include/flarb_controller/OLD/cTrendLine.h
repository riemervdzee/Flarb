#include "flarb_mapbuilder/MapImage.h"


class cTrendLine
{
public:
	int Create(flarb_mapbuilder::MapImage &msg, int x, int y);

	int Slope;
    int Intercept;
    int Start;
    int End;
private:
	int count;
    int xAxisValuesSum;
    int xxSum;
    int xySum;
    int yAxisValuesSum;
	int calculateSlope();
	int calculateIntercept();
	int calculateStart();
	int calculateEnd();
};


