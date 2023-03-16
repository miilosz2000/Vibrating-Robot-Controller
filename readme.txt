To do (future versions): 
	add threshold angle for finding actual orientation : L124 (V3.0) P:VH
    	investigate why angle can be beyond 1? is this a concern? P:M
    	investigate failures, and provide output values to text file (maybe useful?) P:M/L
	do we remove GetSpace n variable? Not part of state anymore, could slow down. inRangeLocation does the same action, but better  P:L
	consider currrent excepts, if any should terminate the program P: H
	


/////Update 1: 
    	Add comments for sections, remove arbitary variables (rewardProduct) 
    	Updated reward for if current distance is not closer than closest in that episode (prevent exploration fear)



