const float errorMargin = 0.05;

struct coinReading{
	float df; //frequency change of M sensor
	float da; //amplitude change of M sensor
	float dv; //frequency change of E sensor
};

struct coinSave{
	uint8_t coinID; //coin ID
	char[10] currencyName; //currency name
	float value; //coin value
	float df; //frequency change of M sensor
	float da; //amplitude change of M sensor
	float dv; //frequency change of E sensor
	
}

void setErrorMargin(float newErrorMargin){
	errorMargin = newErrorMargin;
	return;
}

void detectCoin()