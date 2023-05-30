
#define EFreqWeight 1.f
#define MFreqWeight 1.f
#define MAmplWeight 1.f
#define criticalCertainty 0.9f

void main(){
	
	float EFWeight, MFWeight, MAWeight;
	
	
	//how much each coef impacts total probability
	EFWeight = EFreqWeight / (EFreqWeight + MFreqWeight + MAmplWeight);
	MFWeight = MFreqWeight / (EFreqWeight + MFreqWeight + MAmplWeight);
	MAWeight = MAmplWeight / (EFreqWeight + MFreqWeight + MAmplWeight);
		
	setup();
	
	
	
	while(1){
		
			if(EFreqRDY){
				EFreqCal();
			}
		
			if(MFreqRDY){
				MFreqCal();
			}	

			if(MAmplRDY){
				MAmplCal();
			}
			
			if(coinInserted){
	
			if(EFreqRDY){
				EFreqCal();
			}
		
			if(MFreqRDY){
				MFreqCal();
			}

			if(MAmplRDY){
				MAmplCal();
			}	
			
			
			
			
			
		}
		
		
	}
	
	
	
}