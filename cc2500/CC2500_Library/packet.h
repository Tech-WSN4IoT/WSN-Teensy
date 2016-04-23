class packet{

char pack;

public:
packet(char len, char nodeName, char type, char data){
  pack = len + nodeName + type + data;
}
char meta(){
  return pack[0] + pack[1] + pack[2];
}
char length(){
  return pack[0];
}
char nodeName(){
  return pack[1];
}
char type(){
  return pack[2];
}
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
  
}

