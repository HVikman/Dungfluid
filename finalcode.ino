#include <GyverOLED.h>    //kirjasto oled näytön ohjaukseen
#include <BMP280_DEV.h>   //kirjasto lämpötila ja ilmanpaine antureille
#include <math.h>
#include <movingAvg.h>

movingAvg avgSnd(100); // Jatkuvan aanen keskiarvoistamiseen, huom kirjasto
movingAvg avgTrig(10); // Jatkuvan aanen keskiarvoistamiseen, huom kirjasto (Trigger)

GyverOLED<SSH1106_128x64> oled; //tehdään oled objekti gyveroled kirjastoa käyttäen
BMP280_DEV bmp280_1(10);    //tehdään bmp280_dev objekti ja asetetaan toimimaan spi D9 pinnissä
BMP280_DEV bmp280_2(9);     //tehdään bmp280_dev objekti ja asetetaan toimimaan spi D9 pinnissä

const static uint8_t icons_8x8[][8] PROGMEM = { //taulukko ikoneista
  {0x08, 0x0c, 0x0e, 0xff, 0xff, 0x0e, 0x0c, 0x08}, //0 ylös
  {0x10, 0x30, 0x70, 0xff, 0xff, 0x70, 0x30, 0x10}, //1 alas
  {0xFF, 0x81, 0xFF, 0x00, 0xFF, 0x18, 0x24, 0xC3}, //2 ok
  {0x84, 0x8A, 0x95, 0x84, 0x84, 0x84, 0x48, 0x30}, //3 takaisin
  {0x00, 0x00, 0xff, 0x7e, 0x3c, 0x18, 0x00, 0x00}, //4 nuoli oikealle
  {0x00, 0x00, 0x18, 0x3c, 0x7e, 0xff, 0x00, 0x00},  //5 nuoli vasemmalle
  {0x00, 0x00, 0x88, 0xFB, 0xFB, 0x80, 0x00, 0x00}  //6 info
};

volatile int switchState = LOW;     // Interrupt-tila jonka painike muuttaa
volatile unsigned long last_interrupt_time;
volatile unsigned long interrupt_time;
volatile byte lippu = 0;
volatile int valittu = -1;
volatile bool painettu = 0;
volatile bool testi = 0;
volatile byte ajanotto = 0;

const byte button1 = 19; //napit, ylös
const byte button2 = 18; //alas
const byte button3 = 2;  //valinta
const byte button4 = 3;  //peruutus

float lampo1, paine1, lampo2, paine2, turha, perotus, lerotus; // muuttujat BMP280 antureiden datalle ja infoaliohjelmien kaavoille
float versio = 1.0;  // versio tulostetaan käynnistysintrossa ja lepotilassa

int pinfo, linfo; // paineen info ohjaus, lämmön info ohjaus
int arvo;       // suoraan ADC-muuntimelta tuleva luku
int avgT;   // calculate the moving average
int avg;   // calculate the moving average
int erotus; // Tama tulostetaan naytolle
int graafibuffer[108];  // äänenpaineen graafille
byte paikka = 0; //muuttuja äänigraafin taulukolle

void setup() {
  bmp280_1.begin();                                 // BMP280 alustuskomennot kirjaston mukaan, lepotila
  bmp280_2.begin();
  bmp280_1.setTimeStandby(TIME_STANDBY_2000MS);     // 2 sec standby
  bmp280_2.setTimeStandby(TIME_STANDBY_2000MS);
  bmp280_1.startNormalConversion();                // Jatkuva muunnos päälle
  bmp280_2.startNormalConversion();

  Serial.begin(9600);
  oled.init();

  //asetetaan näppäinkeskeytyksen pinnit, käyttää arduinon sisäistä vastusta jolloin napit voidaan kytkeä ilman vastusta
  pinMode(button1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button1), laskelippu, FALLING);
  pinMode(button2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button2), nostalippu, FALLING);
  pinMode(button3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button3), valinta, FALLING);
  pinMode(button4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button4), peruutus, FALLING);

  avgSnd.begin(); // Kaynnistetaan jatkuva keskiarvoistus
  avgTrig.begin(); // Kaynnistetaan jatkuva keskiarvoistus painiketta varten

  noInterrupts(); // disable all interrupts
  TCCR1A = 0;  // ajastinrekisterin nollaus
  TCCR1B = 0;  // ajastinrekisterin nollaus
  TCNT1 = 0;   // ajastinrekisterin nollaus
  OCR1A = 31249; // keskeytys 2 sekunnin välein
  TCCR1B = TCCR1B | (1 << WGM12); // CTC mode
  TCCR1B = TCCR1B | (1 << CS12);
  TCCR1B = TCCR1B | (1 << CS10); // CS12 ja CS10 = prescaler 1024
  TIMSK1 = TIMSK1 | (1 << OCIE1A); // enable timer compare interrupt
  ADMUX &= B11011111;  // ADC rekisterit "right adjusted", luetaan yhdessä ADCL
  ADMUX |= B01000000;  // AVcc with external capacitor at AREF pin
  ADMUX &= B11110000;  // ADC0(Arduinon A0) portti käytössä
  ADCSRA |= B10000000; //ADC päälle
  ADCSRA |= B00000111; //adc prescaler128. hidastetaan kellonopeutta
  ADCSRA |= B00001000; //laitetaan päälle interrupt kun adc muunnos valmis
  interrupts(); // enable all interrupts
}

void loop() {
  tervetulo();  //ensin tervetuloa ikkuna, tämä näytetään vain kerran
  printMenu();  //siirrytään menu funktioon, jossa on käytännössä koko ohjelma
  
  if (ajanotto >= 30) {
    valittu = 9; //siirrytään lepotilaan kun määräaika täyttyy (ajanotto * 2 sekuntia)
  }

  if (bmp280_1.getMeasurements(lampo1, paine1, turha)) {}
  if (bmp280_2.getMeasurements(lampo2, paine2, turha)) {
      lerotus = lampo1-lampo2;   // sisä ja ulkolämpötilan erotus
      perotus = (((paine1 - paine2) / paine2) * 100);} // kaava jolla päätellään onko paine positiivinen vai negatiivinen, muunnos %
}

void printMenu() {
  testi = 1; //estetään tervetulo ruudun ilmestyminen uudestaan

  switch (valittu) {
  case -1:
    oled.clear();
    oled.setCursor(0, 10);    // näytön grafiikkojen asettelu ja valikon tulostus
    ikoni(0);
    oled.setCursor(0, 20);
    ikoni(1);
    oled.setCursor(110, 10);
    ikoni(2);
    oled.setCursor(110, 20);
    ikoni(3);
    oled.setCursor(18, lippu * 3 + 1);
    ikoni(4);
    oled.setCursor(94, lippu * 3 + 1);
    ikoni(5);

    oled.setScale(1);
    oled.setCursor(28, 1);
    oled.print("Temp");
    oled.setCursor(28, 4);
    oled.print("Pressure");
    oled.setCursor(28, 7);
    oled.print("Sound");
    break;

  case 0:        // Lämmön tulostus, sisä ja ulkolämpö
    oled.clear();
    oled.setScale(2);
    oled.setCursor(0, 0);
    oled.println("Temperature");
    oled.setScale(1);
    oled.println("Case:");
    oled.print(lampo1);
    oled.print((char)247);
    oled.println("C");
    oled.println("Room:");
    oled.print(lampo2);
    oled.print((char)247);   //  celsiuspallura
    oled.print("C");

        if (lerotus < 5){   // mittauksien perusteella arvioidut raja-arvot
        linfo=0;        // linfo ohjaa minkälainen ohje tai päätelmä tulostetaan käyttäjälle
        }           // HOX nykyiset raja-arvot on vain kahdesta hyvin samantyyppisestä PC:stä  mittaamalla haetut
        else if (lerotus >= 5 && lerotus <= 10){
        linfo=1;
        }
        else if (lerotus > 10){
        linfo=2;
        }

    oled.setCursor(110, 20);  // piirretään grafiikat kuvaamaan lisätietoa ja exit nappeja
    ikoni(6);
    oled.setCursor(110, 30);
    ikoni(3);
    break;
    
  case 1:         // ilmanpaineen tulostus
    oled.clear();
    oled.setCursor(0, 0);
    oled.setScale(2);
    oled.println("Pressure");
    oled.setScale(1);
    oled.println("Case:");
    oled.print(paine1);
    oled.println("hPa");
    oled.println("Room:");
    oled.print(paine2);
    oled.println("hPa");
    oled.println("Difference %");
    oled.print(perotus);  // sisäisen ja ulkoisen paineen erotuksen tulostus
    oled.print("%");
    
    if (perotus >= 2) {   // omien mittauksien perusteella haettujen raja-arvojen perusteella
      pinfo = 2;      // asetetetut arvot. Haettu ns mahdollisia rajapintoja arvoille.
      oled.print(" Positive");  // esim minkälainen positiivinen arvo on saavutettavissa
    }       // HOX nykyiset raja-arvot on vain kahdesta hyvin samantyyppisestä pc:stä mitatut
    else if (perotus >= 0 && perotus < 1) {
      pinfo = 1;
      oled.print(" Neutral");
    }
    else if(perotus < 0) {
        pinfo = 0;
      oled.print(" Negative");
    }

    oled.setCursor(110, 20);   // lisätietoa ja exit grafiikat
    ikoni(6);
    oled.setCursor(110, 30);
    ikoni(3);
    break;

  case 2:
    oled.clear();
    oled.setCursor(0, 0);
    oled.setScale(2);
    oled.println("Sound");
    oled.setScale(1);
    while (valittu == 2) {
      graafi();
      aani();
    }
    
    oled.setCursor(110, 20);
    ikoni(2);
    oled.setCursor(110, 30);
    ikoni(3);
    break;

  case 3:    //äänen lisäinfo
    oled.clear();
    oled.setCursor(0, 0);
    oled.setScale(2);
    oled.println("Sound");
    oled.setScale(1);
   oled.println("Follow the graph and ");
    oled.println("value in different");
    oled.println(" scenarios");
    
    oled.setCursor(110, 30);
    ikoni(3);
    break;
    
  case 5:         // ilmanpaineen lisäinfolle
    oled.clear();
    oled.setCursor(0, 0);
    oled.setScale(1);
    
    oled.autoPrintln(true);  // automaattinen täyttötoiminto, hain silti kyllä itse sopivat rivitykset
    if (pinfo == 2) {
      oled.println("Positive pressure");
      oled.println("+Less dust");
      oled.println("+Likely good cooling");
      oled.println("-Can be loud");
      oled.println("->Try less intake");
    }
    if (pinfo == 1) {     // eri tiloille kirjoitettu ohjeet tai kommentit
      oled.println("Neutral pressure");
      oled.println("+Optimal cooling");
      oled.println("+Not unnececcary loud");
      oled.println("->Try if stays through the");
      oled.println("whole rpm range of fans");
    }
    if (pinfo == 0) {     // johtopäätökset eivät vastaa välttämättä tieteellisiä faktoja vaan toimivat esimerkkeinä
      oled.println("Negative pressure");
      oled.println("+Can be most quiet");
      oled.println("-Worse cooling performance");
      oled.println("-Collects more dust");
      oled.println("Try more intake less exhaust");
    }
    oled.autoPrintln(false);
    break;

    case 6:  // lämpötilan lisäinfot
oled.clear();
oled.setCursor(0,0);
oled.setScale(1);
oled.autoPrintln(true);

if(linfo==0){    // omien mittauskohde PC:iden toiminnan perusteella kirjoitetut johtopäätökset
oled.println("Air inside your");
oled.println("case stays really ");
oled.println("cool. Make sure you ");
oled.println("do measurements du- ");
oled.println("ring CPU and GPU ");
oled.println("sress tests.");
}
if(linfo==1){
oled.println("The air temperature inside your");
oled.println("case is under control. Try doing a");
oled.println("longer test while gaming or stress");
oled.println("testing and see if temperatures rise.");
}
if(linfo==2){
oled.println("Air temperature inside the case is");
oled.println("getting really warm. This can be due");
oled.println("to poor fan settings or not enough vent-");
oled.println("ilation. Set a more agressive fan curve");
}
oled.autoPrintln(false);
break;

  case 9:                      // “LEPOTILA” tai ns passiivinen tila
    oled.clear();
    oled.setCursor(0, 0);
    oled.setScale(2);
    oled.println("DUNGFLUID");
    oled.setScale(3);
    oled.print("V");
    oled.print(versio);
    oled.update();
    break;
  }
  oled.update();
}

void nostalippu() { //ylös näppäinpainalluksella nostetaan lippu muuttujan arvoa yhdellä valikon navigointia varten
  ajanotto = 0;
  last_interrupt_time = 0;
  interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 500)
  {
    if (painettu == 0) {
      if (valittu == -1) {
        if (lippu < 2) {
          lippu = lippu + 1;
          painettu = 1;
          delay(200);
          painettu = 0;
        }
      }
    }
  }
  last_interrupt_time = interrupt_time;
}

void laskelippu() { //alas näppäinpainalluksella lasketaan lippu muuttujan arvoa yhdellä valikon navigointia varten
  ajanotto = 0;
  last_interrupt_time = 0;
  interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 500)
  {
    if (painettu == 0) {
      if (valittu == -1) {
        if (lippu > 0) {
          lippu = lippu - 1;
          painettu = 1;
          delay(200);
          painettu = 0;
        }
      }
    }
  }
  last_interrupt_time = interrupt_time;
}

void valinta() { //ok näppäin päävalikossa vaihtaa lippu muuttujan valittu muuttujaksi tai jossain valikon ohjelmassa vaihtaa kyseisen valikon infon
  ajanotto = 0;
  last_interrupt_time = 0;
  interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 500)
  {
    if (painettu == 0) {  // paritetaan eri caset niiden omien infoceissien kanssa jolloin “enter” nappia painettaessa liikutaan niiden välillä
        if (valittu == 2) {  // äänen paritus infocaseensa
          valittu = 3;
        }
        else if (valittu == 1) {   // ilmanpaineen paritus infocaseensa
          valittu = 5;
        }
        else if(valittu==0){valittu=6;}     // lämmön paritus infocaseensa
        else {
          valittu = lippu;
        }
        painettu = 1;
        delay(200);
        painettu = 0;
      }
    }
  last_interrupt_time = interrupt_time;
}

void peruutus() { //vaihtaa valittu muuttujan arvoksi -1 mikä palauttaa ohjelman päävalikkoon
  ajanotto = 0;
  last_interrupt_time = 0;
  interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 500)
  {
   if (painettu == 0) {
      valittu = -1;
      painettu = 1;
      delay(200);
      painettu = 0;
    }
  }
  last_interrupt_time = interrupt_time;
}

void ikoni(byte index) {    //piirretään ikoni taulukosta
  size_t s = sizeof icons_8x8[index];
  for (unsigned int i = 0; i < s; i++) {
    oled.drawByte(pgm_read_byte(&(icons_8x8[index][i])));
  }
}

void tervetulo() { // Käynnistys/latausruutu/Intro
  if (testi == 0) {
    oled.clear();
    oled.setCursor(0, 0);
    oled.setScale(2);
    oled.println("DUNGFLUID");
    oled.setScale(1);
    oled.println("Kurlin, Nieminen,");
    oled.println("Rantanen, Vikman");
    oled.print("           V");
    oled.print(versio);
    oled.println();
    oled.update();
    
    oled.fastLineH(40, 1, 128);  // latausanimaatio
    oled.fastLineV(1, 40, 60);
    oled.fastLineV(127, 40, 60);
    oled.fastLineH(60, 1, 128);
    for (int i = 0; i < 128; i += 2) {
      oled.fastLineV(i, 40, 60);
      oled.fastLineV(i + 1, 40, 60);
      oled.update();
    }
  }
}

void graafi() {    // Äänenpaineen dataan perustuva graafi
int analogVal = avg;
  int xPos = 0;
  graafibuffer[paikka++] = analogVal;
  if (paikka >= 108) {
    paikka = 0;
  }
  for (int i = paikka; i < 108; i++) {   //liikkuvan graafin ohjaus
    int analogVal = graafibuffer[i];
    viiva(xPos, analogVal);
    xPos++;
  }
  for (int i = 0; i < paikka; i++) {
    int analogVal = graafibuffer[i];
    viiva(xPos, analogVal);
    xPos++;;
  }
  oled.setScale(2);
  oled.setCursor(90, 0);
  int tulostus = map(analogVal, 0, 1023, 0, 999);  // äänenpaineen arvon tulostus
  oled.print(tulostus);
  oled.fastLineH(15, 0, 128);
  oled.fastLineV(108, 15, 64);
  
  oled.setCursor(110, 30);
  ikoni(3);
  oled.setCursor(110, 20);
  ikoni(6);
  oled.update();
}

void viiva(int xPos, int analogVal) {     //piirretään viivoja äänigraafia varten
  int lineHeight = map(analogVal, 0, 1023, 0, 48);
  int yPos = 64 - lineHeight;
  oled.fastLineV(xPos, 64, 16, 0);
  oled.fastLineV(xPos, 64, yPos);
}

void aaninappi(){
avgT = avgTrig.reading(arvo); //Painiketta painettaessa tallennetaan keskiarvoa
}

void aani(){
    ADCSRA |= B01000000;
    avg = avgSnd.reading(arvo); // Jatkuvaa keskiarvotallennusta
   // Serial.print("\n avgSnd arvo: ");  // DEBUG: Tama tulostaa juoksevan keskiarvon
   // Serial.print(avg);
  delay(10);}

ISR(ADC_vect){arvo = ADCL | (ADCH << 8);       // arvo = ad muunnos
}

ISR(TIMER1_COMPA_vect) {ajanotto++;}//käytetään ajastinkeskeytystä ajan mittaamiseen lepotilaa varten
