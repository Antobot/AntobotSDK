// Copyright (c) 2023, ANTOBOT LTD.
// All rights reserved.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/************************************************************************************************************************/

//Description:   This is a arduino script receive info from xavier and send command to HMI screen.
//Interface:     serial port
//Inputs:        
//Contact:     zhuang.zhou@antobot.ai

/************************************************************************************************************************/


//variable claim
int up = 10;
int okay = 11;
int down = 12;
int upstate_pre;
int okaystate_pre;
int downstate_pre;

int buttonsum;
int button_sum;
String X2A_uBat_str;
String SoCstr;
int SoC;




bool X2A_Status;
bool newData = false; //flag for new data read from Xavier

//Xavier to Arduino comminication variables
String strA2XMsg;
int A2X_header1 = 56;
int A2X_header2 = 89;
bool A2X_bPower;
String A2X_bPower_str;

int sum;
int A2X_Checksum;

String X2A_Msg;
int X2A_Header1;
int X2A_Header2;
int X2A_bPower;
int X2A_uBat = 100;
int X2A_uBat_past=100;
bool flash_page_battery;
bool flash_page_text;
int X2A_Checksum;


//list all the page name for case
enum pages {boot,logo,battery, power01,power02,power10
           };
unsigned char current_page;
unsigned char last_page;
String pageID;


void setup() {
  // initialization code that only run once:
  Serial.begin(57600);//Xavier to Arduino
  Serial.setTimeout(0);//no waiting time between rw
  Serial1.begin(9600); //Arduino to screen 
  Serial1.setTimeout(0);
  Serial2.begin(9600);//debug serial 
  pinMode(up, INPUT_PULLUP);  //configure pin as input with pullup enabled
  pinMode(okay, INPUT_PULLUP);  //configure pin as input with pullup enabled
  pinMode(down, INPUT_PULLUP);  //configure pin as input with pullup enabled


  upstate_pre = 1;
  okaystate_pre = 1;
  downstate_pre = 1;

  X2A_Status = false;
  Serial1.print("page boot\xFF\xFF\xFF");
  Serial.print("385906f\n");
  delay(6000);
  while (Serial.available () == 0)
  {
    Serial1.print("page logo\xFF\xFF\xFF");
    Serial.print("385906f\n");
    delay(3750);
  }
  X2A_Status = false;
  readXavier();
  while (X2A_Header1!=38 or X2A_Header2!=59){
    Serial1.print("page logo\xFF\xFF\xFF"); 
    delay(3750);
    }

  SoCstr = String(X2A_uBat);
  X2A_uBat_past=X2A_uBat;

  current_page = battery;
  sendtxtpagecmd(current_page, SoCstr);

  A2X_bPower = false;

}

void loop() {
  button_sum = ButtonPress();

  readXavier();
   Serial2.print(current_page);
  //delay(10);
  if (newData == true) {
    newData = false;
  }

  
  if (X2A_uBat != X2A_uBat_past &&X2A_uBat != 0){
    flash_page_battery = true;
    flash_page_text = true;
    X2A_uBat_past = X2A_uBat;
    SoCstr = String(X2A_uBat);
    
    }
  else{
    flash_page_battery = false;
    }

  //Serial2.print(SoCstr);
  
  if (button_sum > 0)
  {
    switch (current_page)
    {

      case Power01:
        if (button_sum == 100)
        {
          current_page = Power02;
          sendpagecmd(current_page);
        }
        else if (button_sum == 1)
        {
          current_page = Power02;
          sendpagecmd(current_page);
        }
        else if (button_sum == 10)
        {

          current_page = Power10; //when there is no serial input, then go to next page:Power11
          sendpagecmd(current_page);
        }
        break;

      case Power02:
        if (button_sum == 100)
        {
          current_page = Power01;
          sendpagecmd(current_page);
        }
        else if (button_sum == 1)
        {
          current_page = Power01;
          sendpagecmd(current_page);
        }
        else if (button_sum == 10)
        {
          A2X_bPower = true;
          sendpagecmd(current_page);
        }
        break;
        
    }
  }
  else //no button input, but xavier input
  {
    if (flash_page_battery == true){
      Serial1.print("t0.txt=\"" + SoCstr + "\"\xFF\xFF\xFF"); //text of soc
      int SoC_int = SoCstr.toInt();
      Serial1.flush();
      flash_page_battery = false;
    }

     if (X2A_bPower == 1)  //shutdown request from any source
        {
          current_page = power10;
          sendpagecmd(current_page);
          }
    }
  }


  //generate message and send to Xavier
  if (A2X_bPoweer == true)
    A2X_bPower_str = "1";
  else
    A2X_bPower_str = "0";




  sum = A2X_header1 + A2X_header2 + A2X_bPower;
  A2X_Checksum = 256 - sum % 256;
  strA2XMsg = String(A2X_header1, HEX) + String(A2X_header2, HEX)  + A2X_bPower_str+ String(A2X_Checksum, HEX) + '\n';

  //delay(10);
  Serial2.print(strA2XMsg);
  Serial.print(strA2XMsg);
  Serial2.print("finish print");
  Serial.flush();
}


int ButtonPress() { //function to check which button(es) is being pressed
  buttonsum = 0;
  int upstate = digitalRead(up);
  int okaystate = digitalRead(okay);
  int downstate = digitalRead(down);
  if (upstate == 0 && upstate_pre == 1) //
  {
    buttonsum = buttonsum + 100;
  }

  if (okaystate == 0 && okaystate_pre == 1) //and
  {
    buttonsum = buttonsum + 10;
  }
  if (downstate == 0 && downstate_pre == 1) //and downstate_pre==1
  {
    buttonsum = buttonsum + 1;
  }
  upstate_pre = upstate;
  okaystate_pre = okaystate;
  downstate_pre = downstate;
  delay(10);
  return buttonsum;
}

void sendtxtpagecmd(int pagecmd, String SoC_str) //function to send cmd to go to the specific page
{
  pageID = pagecmd;
  Serial1.print("page ");
  Serial1.print(pageID);
  Serial1.print("\xFF\xFF\xFF");
  int SoC_int = SoC_str.toInt();
  Serial1.print("t0.txt=\"" + SoC_str + "\"\xFF\xFF\xFF"); //text of soc
  Serial1.flush();
}

void sendpagecmd(int pagecmd) //function to send cmd to go to the specific page
{
  pageID = pagecmd;
  Serial1.print("page ");
  Serial1.print(pageID);
  Serial1.print("\xFF\xFF\xFF");
  Serial1.flush();
}

void readXavier()
{
  char X2A_uBat_char[3];

  while (Serial.available() > 0 && newData == false) //if?
  {
    X2A_Status = true;
    X2A_Msg = Serial.readString();
    Serial2.print(X2A_Msg);
    //len = X2A_Msg.length();
    X2A_Header1 = X2A_Msg.substring(0, 2).toInt();
    X2A_Header2 = X2A_Msg.substring(2, 4).toInt();
    X2A_bPower = X2A_Msg.substring(4, 5).toInt();
    X2A_uBat_str = X2A_Msg.substring(5, 7);
    X2A_uBat_str.toCharArray(X2A_uBat_char, 3);
    X2A_uBat = strtoul(X2A_uBat_char, 0, 16);
    newData=true;
    if (X2A_Header1 == 38 && X2A_Header2==59 && X2A_Msg.substring(7, 8)=='\n')
    {
      newData = true;
      break;
    }

  }
}
