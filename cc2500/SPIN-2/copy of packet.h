class Packet{
	char pack[4]; 
	
	public: 
	Packet();
	Packet(char len, char nodeName, char type, char data);
	Packet(char len, char nodeName, char type); 
	
	/*char meta(){
		char meta[3] = {pack[0], pack[1], pack[2]};
		return meta; 
	}	
	char full(){return pack;}
	*/
	
	char length(){return pack[0];}

	char nodeName(){return pack[1];}

	char type(){return pack[2];}
	
	char data(){return pack[3];} //Simplify for now 
	
	void setLen(char in){pack[0] = in;}
	void setnodeName(char in){pack[1] = in;}
	void setType(char in){pack[2] = in;}
	void setData(char in){pack[3] = in;}

	/*char data(){
	  char out = {0x00};
	  if(pack[0] > 3){
		for (int i = 3; i < pack[0]; i++){
			out{i} = pack{i}; 
		}
		return out;
	  }
	  else{
		return [];
	  }
	}*/
	private: 
};
Packet::Packet(){
}

Packet::Packet(char len, char nodeName, char type, char data){
  //pack[] = len + nodeName + type + data;
  //pack = {len, nodeName, type, data};
  pack[0] = len; 
  pack[1] = nodeName; 
  pack[3] = type; 
  pack[4] = data;
}

Packet::Packet(char len, char nodeName, char type){
  //pack[] = len + nodeName + type;
  //pack = {len, nodeName, type, data, 0x00}; 
  pack[0] = len; 
  pack[1] = nodeName; 
  pack[3] = type; 
  pack[4] = 0x00;
/*  pack[0] = len; 
  pack[1] = nodeName; 
  pack[3] = type; 
  pack[4] = 0x00; */ 
}
