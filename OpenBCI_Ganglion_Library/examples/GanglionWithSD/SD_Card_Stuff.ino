
#define BLOCK_5MIN  11000
#define BLOCK_15MIN  33000
#define BLOCK_30MIN  66000
#define BLOCK_1HR  131000
#define BLOCK_2HR  261000
#define BLOCK_4HR  521000
#define BLOCK_12HR  1561000
#define BLOCK_24HR  3122000

#define OVER_DIM 20 // make room for up to 20 write-time overruns


char fileSize = '0';  // SD file size indicator
int blockCounter = 0;

uint32_t BLOCK_COUNT;
SdFile openfile;  // want to put this before setup...
Sd2Card card(SD_SS);// SPI needs to be init'd before here
SdVolume volume;
SdFile root;
uint8_t* pCache;      // array that points to the block buffer on SD card
uint32_t MICROS_PER_BLOCK = 4000; // block write longer than this will get flaged
uint32_t bgnBlock, endBlock; // file extent bookends
int byteCounter = 0;    // used to hold position in cache
//int blockCounter;       // count up to BLOCK_COUNT with this
boolean openvol;
boolean cardInit = false;
boolean fileIsOpen = false;

struct {
  uint32_t block;   // holds block number that over-ran
  uint32_t micro;  // holds the length of this of over-run
} over[OVER_DIM];
uint32_t overruns;      // count the number of overruns
uint32_t maxWriteTime;  // keep track of longest write time
uint32_t minWriteTime;  // and shortest write time
uint32_t t;        // used to measure total file write time

int fileHundreds, fileTens, fileOnes;  // enumerate succesive files on card and store number in EEPROM
char currentFileName[] = "OBCI_000.csv"; // file name will enumerate 000 to 999
char elapsedTime[] = {"\n%Total time mS:\n"};  // 17
char minTime[] = {  "%min Write time uS:\n"};  // 20
char maxTime[] = {  "%max Write time uS:\n"};  // 20
char overNum[] = {  "%Over:\n"};               //  7
char blockTime[] = {  "%block, uS\n"};         // 11    74 chars + 2 32(16) + 2 16(8) = 98 + (n 32x2) up to 24 overruns...
char stopStamp[] = {  "%STOP AT\n"};      // used to stamp SD record when stopped by PC
char startStamp[] = {  "%START AT\n"};    // used to stamp SD record when started by PC


char sdProcessChar(char character) {
    ganglion.gotSerial = false;
    switch (character) {
        case 'A': // 5min
        case 'S': // 15min
        case 'F': // 30min
        case 'G': // 1hr
        case 'H': // 2hr
        case 'J': // 4hr
        case 'K': // 12hr
        case 'L': // 24hr
        case 'a': // 512 blocks
            fileSize = character;
            SDfileOpen = setupSDcard(character);
            break;
        case 'j': // close the file, if it's open
            if(SDfileOpen){
                SDfileOpen = closeSDfile();
            }
            break;
        case 's':
            if(SDfileOpen) {
                stampSD(DEACTIVATE);
            }
            break;
        case 'b':
            if(SDfileOpen) {
                stampSD(ACTIVATE);
            }
            break;
        default:
            break;
    }
    return character;

}


boolean setupSDcard(char limit){

  if(!cardInit){
      if(!card.init(SPI_HALF_SPEED, SD_SS)) {
        if(!ganglion.is_running) {
          ganglion.loadString("initialization failed. Things to check:",39,false);
          ganglion.loadString("* is a card is inserted?",24,true);
        }
      //    card.init(SPI_FULL_SPEED, SD_SS);
      } else {
        if(!ganglion.is_running) {
          ganglion.loadString("Wiring is correct and a card is present.",40,true);
        }
        cardInit = true;
      }
      if (!volume.init(card)) { // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
        if(!ganglion.is_running) {
          ganglion.loadString("Could not find FAT16/FAT32 partition. Make sure you've formatted the card",73,true);
        }
        return fileIsOpen;
      } else {
        if(!ganglion.is_running) {
          ganglion.loadString("Card Initialized",16,true);
        }
      }
   }

  // use limit to determine file size
  switch(limit){
    case 'h':
      BLOCK_COUNT = 50; break;
    case 'a':
      BLOCK_COUNT = 256; break;
    case 'A':
      BLOCK_COUNT = BLOCK_5MIN; break;
    case 'S':
      BLOCK_COUNT = BLOCK_15MIN; break;
    case 'F':
      BLOCK_COUNT = BLOCK_30MIN; break;
    case 'G':
      BLOCK_COUNT = BLOCK_1HR; break;
    case 'H':
      BLOCK_COUNT = BLOCK_2HR; break;
    case 'J':
      BLOCK_COUNT = BLOCK_4HR; break;
    case 'K':
      BLOCK_COUNT = BLOCK_12HR; break;
    case 'L':
      BLOCK_COUNT = BLOCK_24HR; break;
    default:
      if(!ganglion.is_running) {
        ganglion.loadString("invalid BLOCK count",19,true);
      }
      return fileIsOpen;
      break;
  }

  openvol = root.openRoot(volume);
  int numFiles = root.ls(0x00);
  incrementFileCounter(numFiles+1);
  openfile.remove(root, currentFileName); // if the file is over-writing, let it!
  if (!openfile.createContiguous(root, currentFileName, BLOCK_COUNT*512UL)) {
    if(!ganglion.is_running) {
      ganglion.loadString("createContiguous fail",21,true);
    }
    cardInit = false;
  } //else{Serial.print("got contiguous file...");delay(1);}
  // get the location of the file's blocks
  if (!openfile.contiguousRange(&bgnBlock, &endBlock)) {
    if(!ganglion.is_running) {
      ganglion.loadString("get contiguousRange fail",24,true);
    }
    cardInit = false;
  } //else{Serial.print("got file range...");delay(1);}
  // grab the Cache
  pCache = (uint8_t*)volume.cacheClear();
  // tell card to setup for multiple block write with pre-erase
  if (!card.erase(bgnBlock, endBlock)){
    if(!ganglion.is_running) {
      ganglion.loadString("erase block fail",16,true);
    }
    cardInit = false;
  } //else{Serial.print("erased...");delay(1);}
  if (!card.writeStart(bgnBlock, BLOCK_COUNT)){
    if(!ganglion.is_running) {
      ganglion.loadString("writeStart fail",15,true);
    }
    cardInit = false;
  } else{
    fileIsOpen = true;
    delay(1);
  }
  digitalWrite(SD_SS,HIGH);  // release the spi
  // initialize write-time overrun error counter and min/max wirte time benchmarks
  overruns = 0;
  maxWriteTime = 0;
  minWriteTime = 65000;
  byteCounter = 0;  // counter from 0 - 512
  blockCounter = 0; // counter from 0 - BLOCK_COUNT;
  if(fileIsOpen == true){  // send corresponding file name to controlling program
    ganglion.writingToSD = true;
      ganglion.loadString("Writing to file: ",17,false);
      for(int i=0; i<12; i++){
        ganglion.loadChar(currentFileName[i],false);
      }
      ganglion.loadNewLine();
  }
  ganglion.prepToSendBytes();
  return fileIsOpen;
}

boolean closeSDfile(){
  if(fileIsOpen){
    digitalWrite(SD_SS,LOW);  // take spi
    card.writeStop();
    openfile.close();
    digitalWrite(SD_SS,HIGH);  // release the spi
    fileIsOpen = false;
    ganglion.writingToSD = false;
    if(!ganglion.is_running){ // verbosity. this also gets insterted as footer in openFile
      ganglion.loadString("Total Elapsed Time: ",20,false); ganglion.loadInt(t,false); ganglion.loadString(" mS",3,true); //delay(10);
      ganglion.loadString("Max write time: ",16,false); ganglion.loadInt(maxWriteTime,false); ganglion.loadString(" uS",3,true); //delay(10);
      ganglion.loadString("Min write time: ",16,false); ganglion.loadInt(minWriteTime,false); ganglion.loadString(" uS",3,true); //delay(10);
      ganglion.loadString("Overruns: ",10,false); ganglion.loadInt(overruns,true);
      if (overruns > 0) {
        uint8_t n = overruns > OVER_DIM ? OVER_DIM : overruns;
        Serial.println("fileBlock\tmicros");
        for (uint8_t i = 0; i < n; i++) {
          ganglion.loadInt(over[i].block,false); ganglion.loadChar('\t',false); ganglion.loadInt(over[i].micro,true);
        }
      }
      // ganglion.sendEOT();
    }
  }else{
    if(!ganglion.is_running) {
      ganglion.loadString("No open file to close",21,true);
      // ganglion.sendEOT();
    }
  }
  ganglion.prepToSendBytes();
  return fileIsOpen;
}

void writeDataToSDcard(byte sampleNumber){
  boolean addComma = true;
  // convert 8 bit sampleCounter into HEX
  convertToHex(sampleNumber, 1, addComma);
  // convert 24 bit channelData into HEX
  for (int currentChannel = 0; currentChannel < 4; currentChannel++){
    convertToHex(ganglion.channelData[currentChannel], 5, addComma);
    if(currentChannel == 2){
      addComma = false;
      if(ganglion.newAuxData || ganglion.newAccelData) {addComma = true;}  // format CSV
    }
  }

  if(ganglion.newAuxData == true){
    // convert auxData into HEX
    for(int currentChannel = 0; currentChannel <  3; currentChannel++){
      convertToHex(ganglion.auxData[currentChannel], 1, addComma);
      if(currentChannel == 1) addComma = false;
    }
    ganglion.newAuxData = false;
  }// end of aux data log
  else if(ganglion.newAccelData == true){  // if we have accelerometer data to log
    // convert 16 bit accelerometer data into HEX
    for (int currentChannel = 0; currentChannel < 3; currentChannel++){
      convertToHex(ganglion.axisData[currentChannel], 1, addComma);
      if(currentChannel == 1) addComma = false;
    }
    ganglion.newAccelData = false;
  }// end of accelerometer data log

   // add aux data logging...
}


void writeCache(){
    if(blockCounter > BLOCK_COUNT) return;
    uint32_t tw = micros();  // start block write timer
    digitalWrite(SD_SS,LOW);  // take spi
    if(!card.writeData(pCache)) {
      // if (!ganglion.is_running) {
        Serial.println("block write fail");
        // ganglion.sendEOT();
      // }
    }   // write the block
    digitalWrite(SD_SS,HIGH); //ganglion.csHigh(SD_SS);  // release spi
    tw = micros() - tw;      // stop block write timer
    if (tw > maxWriteTime) maxWriteTime = tw;  // check for max write time
    if (tw < minWriteTime) minWriteTime = tw;  // check for min write time
    if (tw > MICROS_PER_BLOCK) {      // check for overrun
      if (overruns < OVER_DIM) {
        over[overruns].block = blockCounter;
        over[overruns].micro = tw;
      }
      overruns++;
    }
    byteCounter = 0; // reset 512 byte counter for next block
    blockCounter++;    // increment BLOCK counter
    if(blockCounter == BLOCK_COUNT-1){
      t = millis() - t;
      if(ganglion.stopRunning() == false){
        ganglion.loadString("SD control: stopRunning",23,true);
      }
      writeFooter();
    }
    if(blockCounter == BLOCK_COUNT){
      SDfileOpen = closeSDfile();
      BLOCK_COUNT = 0;
    }  // we did it!
}


 void incrementFileCounter(int numFiles){
  fileOnes = numFiles%10;
  fileTens = (numFiles/10)%10;
  fileHundreds = (numFiles/100)%10;
  currentFileName[5] = fileHundreds + '0';
  currentFileName[6] = fileTens + '0';
  currentFileName[7] = fileOnes + '0';
 }

void stampSD(boolean state){
  unsigned long time = millis();
  if(state){
    for(int i=0; i<10; i++){
      pCache[byteCounter] = startStamp[i];
      byteCounter++;
      if(byteCounter == 512){
        writeCache();
      }
    }
  } else {
    for(int i=0; i<9; i++){
      pCache[byteCounter] = stopStamp[i];
      byteCounter++;
      if(byteCounter == 512){
        writeCache();
      }
    }
  }
  convertToHex(time, 7, false);
}

void writeFooter(){
  for(int i=0; i<17; i++){
    pCache[byteCounter] = elapsedTime[i];
    byteCounter++;
  }
  convertToHex(t, 7, false);

  for(int i=0; i<20; i++){
    pCache[byteCounter] = minTime[i];
    byteCounter++;
  }
  convertToHex(minWriteTime, 7, false);

  for(int i=0; i<20; i++){
    pCache[byteCounter] = maxTime[i];
    byteCounter++;
  }
  convertToHex(maxWriteTime, 7, false);

  for(int i=0; i<7; i++){
    pCache[byteCounter] = overNum[i];
    byteCounter++;
  }
  convertToHex(overruns, 7, false);
  for(int i=0; i<11; i++){
    pCache[byteCounter] = blockTime[i];
    byteCounter++;
  }
  if (overruns) {
    uint8_t n = overruns > OVER_DIM ? OVER_DIM : overruns;
    for (uint8_t i = 0; i < n; i++) {
      convertToHex(over[i].block, 7, true);
      convertToHex(over[i].micro, 7, false);
    }
  }
  for(int i=byteCounter; i<512; i++){
    pCache[i] = NULL;
  }
  writeCache();
}

//    CONVERT RAW BYTE DATA TO HEX FOR SD STORAGE
void convertToHex(long rawData, int numNibbles, boolean useComma){

  for (int currentNibble = numNibbles; currentNibble >= 0; currentNibble--){
    byte nibble = (rawData >> currentNibble*4) & 0x0F;
    if (nibble > 9){
      nibble += 55;  // convert to ASCII A-F
    }
    else{
      nibble += 48;  // convert to ASCII 0-9
    }
    pCache[byteCounter] = nibble;
    byteCounter++;
    if(byteCounter == 512){
      writeCache();
    }
  }
  if(useComma == true){
    pCache[byteCounter] = ',';
  }else{
    pCache[byteCounter] = '\n';
  }
  byteCounter++;
  if(byteCounter == 512){
    writeCache();
  }
}// end of byteToHex converter
