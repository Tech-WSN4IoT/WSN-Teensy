class packet{

char pack;

packet(char len, char nodeName, char type, char data);

public:

char meta(){return pack[0] + pack[1] + pack[2];}

char length(){return pack[0];}

char nodeName(){return pack[1];}

char type(){return pack[2];}

char data(){
  char out = [];
  if pack[0] > 3{
    for (i = 3; i < pack[0]; i++){
      char = out + pack[i];
    }
    return out;
  }
  else{
    return [];
  }
}

packet();
  
};

packet::packet(char len, char nodeName, char type, char data){
  pack = len + nodeName + type + data;
}

packet::packet(char len, char nodeName, char type){
  pack = len + nodeName + type;
}
