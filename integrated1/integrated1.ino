#include <Wire.h>
#include <LiquidCrystal.h>
#include <max6675.h>

//////// Start buttons library
enum ButtonKey
{
  KEY_RIGHT = 0,
  KEY_UP,
  KEY_DOWN,
  KEY_LEFT,
  KEY_SELECT,
  KEY_RESET,
  KEY_NONE,
  KEY_SIZE = KEY_NONE
};

enum ButtonAction
{
  ACTION_NONE = 0,
  ACTION_DOWN,
  ACTION_UP,
  ACTION_CLICK = 4,
  ACTION_LONGCLICK = 8
};

struct ButtonHit
{
  ButtonKey key;
  ButtonAction action;

  static ButtonKey s_lastKey;
  static ButtonKey s_lastNonEmptyKey;
  static ButtonAction s_lastNonEmptyAction;
  static unsigned int s_keyDownSince;
};

ButtonKey ButtonHit::s_lastKey = KEY_NONE;
ButtonKey ButtonHit::s_lastNonEmptyKey = KEY_NONE;
ButtonAction ButtonHit::s_lastNonEmptyAction = ACTION_NONE;
unsigned int ButtonHit::s_keyDownSince = 0;


struct ButtonHit checkButton()
{
  int x;
  x = analogRead (0);

  ButtonKey key = KEY_NONE;
  if (x < 60)
    key = KEY_RIGHT;
  else if (x < 800)
    key = (ButtonKey)(x/200 + 1);

  struct ButtonHit ret;
  ret.key = key;
  ret.action = ACTION_NONE;
  if (ret.s_lastKey == key)
  {
    if (key != KEY_NONE)
      ++ret.s_keyDownSince;
  }
  else
  {
    ret.action = ACTION_UP;

    if (key != KEY_NONE)
    {
      ret.s_lastNonEmptyKey = key;
      ret.action = ACTION_DOWN;
    }
    else
    {
      // for button up we need to have a key
      ret.key = ret.s_lastKey;

      //see if we have click or longclick
      if (ret.s_keyDownSince > 3)
        ret.action = (ButtonAction)(ret.action | ACTION_LONGCLICK);
      else
        ret.action = (ButtonAction)(ret.action | ACTION_CLICK);
    }

    ret.s_lastKey = key;
    ret.s_keyDownSince = 0;
  }

  if (ret.action != ACTION_NONE)
    ret.s_lastNonEmptyAction = ret.action;  

  return ret;
}
/////      End buttons library

//////// Start generic utils
int DECIMAL_1(float x)
{
  return int(round((x-(int)x)*10));
}

int writeTemp(double temp, char *buf)
{
  int offset = sprintf(buf, "%d", (int)temp);
  // one decimal digit
  return offset + sprintf(buf+offset, ".%d", int((temp-int(temp))*10));
}

void fillLine(char *buf)
{
  int i = strlen(buf);
  for (; i<16; ++i)
    buf[i] = ' ';
}


//////// End generic utils


/////      Start common data structures
struct TemperatureStep
{
  short targetTemp; // in C
  short duration;	// seconds
  bool  holdTemp;	// hold or increase 
};

#define MAX_STEPS 32

static struct TemperatureStep s_configuredSteps[MAX_STEPS];
static int s_configuredStepsCount = 0;


void resetSteps(struct TemperatureStep *steps)
{
  int i;
  for (i=0; i<MAX_STEPS; ++i)
  {
    steps[i].targetTemp = 0;
    steps[i].duration = 0;
    steps[i].holdTemp = true;
  }

  s_configuredStepsCount = 0;
}

// temperature sensor goes here
double tempC = 0;
// sliding temp average
static const int AVG_TEMP_SAMPLES = 16;
double tempWindow[AVG_TEMP_SAMPLES] = {0.};
unsigned char nextSamplePos = 0;
double tempAverage = 0;

unsigned int s_ticks = 0;
unsigned long s_secondsAtStep = 0;	// when running steps
unsigned long s_msAtStep = 0;		// ditto
unsigned long s_msSinceStart = 0;	// updated in the loop
unsigned long s_msForCurrentStep = 0;	// mseconds when current step started

/////      End  common data structures


/////      Start SSR control logic
struct PID
{
  float i; // integral part
  bool heatOn;
  bool activateI;
  float tempIncrementRate; // computed increase rate for INCR mode
  float startTemp;         // starting temperature for INC mode
  static const float P_RATIO = 0.2f;
  static const float I_RATIO = 0.0001f;
  static const float D_RATIO = 100.f;	// inertia compensation: 
					// depends on how far the sensor is from the coil
  static const float D_RATIO_NEG = 50.f;// smaller when temperature is decreasing.
};
struct PID s_PID;

void initSSR()
{
  pinMode(3, OUTPUT);
}

void toggleSSR(uint8_t value)
{
  {
    char buf[20] = "off";
    if (value == HIGH) sprintf(buf, "on");
    Serial.print("Turned SSR ");
    Serial.println(buf);
  }

  digitalWrite(3, value);
  s_PID.heatOn = (value == HIGH);
}

void resetPID()
{
  s_PID.i = 0;
  s_PID.activateI = 0;
  s_PID.tempIncrementRate = 0;
  s_PID.startTemp = 0;
}

/////      End   SSR control logic



/////      Start menu logic

enum MenuItems
{
  MENU_MONITOR,
  MENU_EDITSTEPS,
  MENU_RUNSTEPS,
  MENUITEMS_SIZE
};


struct SubMenu
{
  short stepId;
  char vars[64];		// NOTE: make sure to provide enough space to fix MAX_STEPS
};

union EditSubmenu	// maps into vars[16]
{
  struct
  {
    char nextAction; // add/remove/edit
    char subAction;	// for add/edit, it is edit of step type, target temperature and time to reach
    bool executingAction; // whether executing action above
    int  stepsCount;	// count of steps stored
    struct TemperatureStep steps[MAX_STEPS];	
  } 
  varsNamed;
  char vars[64];
};

enum EditSubmenuActions
{
  EDITACTION_ADD = 0,
  EDITACTION_REMOVE,
  EDITACTION_EDIT
};

enum EditSubmenuSubActions
{
  EDITSUBACTION_TYPE = 0,
  EDITSUBACTION_TEMP,
  EDITSUBACTION_DURATION
};


struct Menu
{
  int submenuId;
  struct SubMenu subMenus[MENUITEMS_SIZE];
};



void resetStepsRun(struct Menu * menu)
{
  menu->subMenus[MENU_RUNSTEPS].stepId = 0;
  s_msForCurrentStep = s_msSinceStart;
  s_PID.heatOn = false;
  resetPID();
  // disable heating element for precaution.
  toggleSSR(LOW);
}

void initStep(struct Menu * menu)
{
  int stepId = menu->subMenus[MENU_RUNSTEPS].stepId;

  resetPID();

  if (stepId >= s_configuredStepsCount)
    return;

  s_msForCurrentStep = s_msSinceStart;

  if (!s_configuredSteps[stepId].holdTemp)
  {
    // set initial temp and temp. increase rate
    s_PID.startTemp = tempC;
    struct TemperatureStep * step = &s_configuredSteps[stepId];
    s_PID.tempIncrementRate = (step->targetTemp - s_PID.startTemp) / (step->duration * 60.f);
  }
}

// returns target temp
float runStep(struct Menu * menu)
{
  short stepId = menu->subMenus[MENU_RUNSTEPS].stepId;
  float targetTemp = 0.f;

  static float prevAverageTemp = -1.f;

  // actually perform running here
  s_msAtStep = (s_msSinceStart - s_msForCurrentStep);
  s_secondsAtStep = s_msAtStep / 1000;

  // do the PID stuff
  if (stepId < s_configuredStepsCount)
  {
    struct TemperatureStep * curStep = &s_configuredSteps[stepId];

    targetTemp = curStep->targetTemp;
    if (!curStep->holdTemp)
    {
      float stepDuration = s_msAtStep / 1000.f;
      targetTemp = s_PID.startTemp + stepDuration * s_PID.tempIncrementRate;
    }

    float dT = targetTemp - tempC;
    float p = dT * PID::P_RATIO;
    if (s_PID.activateI)
      s_PID.i += dT * PID::I_RATIO;
    else if(tempC >= targetTemp)
      s_PID.activateI = true;

    float d = 0.f;
    if (prevAverageTemp > -1.f)
    {
      float deltaT = (tempAverage - prevAverageTemp); // counter inertia 
      if (deltaT < 0)
        d = -deltaT * PID::D_RATIO_NEG; // sensor may report a sporadic decrease even if temp is actually increasing
					// hence NEG ratio should be less.
      else
        d = -deltaT * PID::D_RATIO;
    }
    prevAverageTemp = tempAverage;
    

    if (0)
    {
      char buf[40];
      sprintf(buf, "p=%d.%d, i=%d.%d, dT=%d.%d, tempC=%d, targetT=%d", (int)p, DECIMAL_1(p), (int)s_PID.i, DECIMAL_1(s_PID.i),
      (int)dT, DECIMAL_1(dT), (int)tempC, curStep->targetTemp );
      Serial.println(buf);
    }

    int action = p + s_PID.i + d;
    //if (action >= 1 && !s_PID.heatOn)
    if (action > 0.1 && !s_PID.heatOn)
    {
      toggleSSR(HIGH);
    }
    else if (action < 1 && s_PID.heatOn)
    {
      toggleSSR(LOW);
    }

    if (s_secondsAtStep >= ((unsigned long)curStep->duration)*60)
    {
      // move to next step.
      ++stepId;
      if (stepId < s_configuredStepsCount)
      {
        menu->subMenus[MENU_RUNSTEPS].stepId = stepId;
        initStep(menu);
      }
    } 
  }

  if (stepId >= s_configuredStepsCount)
  {
    // disable heating element.
    toggleSSR(LOW);
    menu->submenuId = MENU_MONITOR;
  }

  return targetTemp;
}

void getCurrentMessage(struct Menu * menu, char **firstLine, char **secondLine)
{
  static char buf1[17], buf2[17];
  static const char * s_editActions[3] = { 
    "ADDSTEP", "RMSTEP", "EDITSTEP"           };
  memset(buf1, 0, 17);
  memset(buf2, 0, 17);
  switch(menu->submenuId)
  {
  case MENU_MONITOR:
    sprintf(buf1, "[M] ");
    writeTemp(tempC, buf1+4);
    strcat(buf1, "C  ");
    fillLine(buf1);
    *firstLine = buf1;
    *secondLine = (char*)" <           >E ";
    break;	
  case MENU_EDITSTEPS:
    {
      EditSubmenu * sub = (EditSubmenu*)menu->subMenus[menu->submenuId].vars;
      short stepId = menu->subMenus[MENU_EDITSTEPS].stepId;
      short stepToDisplay = min(stepId+1, sub->varsNamed.stepsCount);
      sprintf(buf1, "[E] %d/%d ", stepToDisplay, sub->varsNamed.stepsCount);
      strcat(buf1, s_editActions[(int)sub->varsNamed.nextAction]);
      fillLine(buf1);
      *firstLine = buf1;
      if (!sub->varsNamed.executingAction)
      {
        *secondLine = (char*)"M<           >R ";
      }
      else
      {
        switch(sub->varsNamed.subAction)
        {
          int ofs;
        case EDITSUBACTION_TYPE:
          sprintf(buf2, "Type: " );
          strcat(buf2, sub->varsNamed.steps[stepId].holdTemp ? "HOLD" : "INCR");
          break;
        case EDITSUBACTION_TEMP:
          ofs = sprintf(buf2, "Tgt Temp: " );
          sprintf(buf2+ofs, "%dC", sub->varsNamed.steps[stepId].targetTemp);
          break;
        case EDITSUBACTION_DURATION:
          ofs = sprintf(buf2, "Time: " );
          sprintf(buf2+ofs, "%dm", sub->varsNamed.steps[stepId].duration);
          break;
        }
        fillLine(buf2);
        *secondLine = buf2;
      }
    }
    break;	
  case MENU_RUNSTEPS:
    {
      short stepId = menu->subMenus[MENU_RUNSTEPS].stepId;
      if (stepId >= s_configuredStepsCount)
      {
        menu->submenuId = MENU_MONITOR;
        return;
      }

      float targetTemp = runStep(menu);

      int i;
      short stepToDisplay = min(stepId+1, s_configuredStepsCount);
      int pos = sprintf(buf1, "[R] %d/%d ", stepToDisplay, s_configuredStepsCount);
      if (stepId < s_configuredStepsCount)
      {
        struct TemperatureStep * curStep = &s_configuredSteps[stepId];
        pos += sprintf(buf1+pos, "%ld/%dm", s_secondsAtStep/60, curStep->duration);
        if (!curStep->holdTemp && pos < 14)
          pos += sprintf(buf1+pos, " I");
      }
      fillLine(buf1);
      *firstLine = buf1;

      sprintf(buf2, "E< ");
      writeTemp(tempC, buf2+3);
      if (stepId < s_configuredStepsCount)
      {
        struct TemperatureStep * curStep = &s_configuredSteps[stepId];
        int pos = strlen(buf2);
        if (curStep->holdTemp)
          sprintf(buf2+pos, "/%d", curStep->targetTemp);
        else
          sprintf(buf2+pos, "/%d", (int)round(targetTemp));
      }
      strcat(buf2, "C");	

      for (i=strlen(buf2); i<13; ++i)
        buf2[i] = ' ';
      strcat(buf2, ">M ");
      *secondLine = buf2; 
    }
    break;	
  }
}

void activateStepsConfiguration(struct Menu * menu)
{
  // copy data to s_configuredSteps
  EditSubmenu * sub = (EditSubmenu*)menu->subMenus[MENU_EDITSTEPS].vars;
  s_configuredStepsCount = sub->varsNamed.stepsCount;
  memcpy(s_configuredSteps, sub->varsNamed.steps, sizeof(sub->varsNamed.steps));
}

void handleButtonsMonitor(struct Menu * menu, struct ButtonHit buttonHit)
{
  switch(buttonHit.key)
  {
  case KEY_RIGHT:
    menu->submenuId = MENU_EDITSTEPS;
    break;
  case KEY_UP:
    break;
  case KEY_DOWN:
    break;
  case KEY_LEFT:
    break;
  case KEY_SELECT:
    break;
  default:
    break;
  }
}

void handleButtonsEditSteps(struct Menu * menu, struct ButtonHit buttonHit)
{
  EditSubmenu * sub = (EditSubmenu*)menu->subMenus[menu->submenuId].vars;
  int stepId = menu->subMenus[menu->submenuId].stepId;

  if (!sub->varsNamed.executingAction)
  {
    switch(buttonHit.key)
    {
    case KEY_RIGHT:
      activateStepsConfiguration(menu);
      resetStepsRun(menu);
      initStep(menu);
      menu->submenuId = MENU_RUNSTEPS;
      break;
    case KEY_UP:
      --sub->varsNamed.nextAction;
      if (sub->varsNamed.nextAction <0)
        sub->varsNamed.nextAction = 2;
      break;
    case KEY_DOWN:
      ++sub->varsNamed.nextAction;
      if (sub->varsNamed.nextAction >2)
        sub->varsNamed.nextAction = 0;
      break;
    case KEY_LEFT:
      menu->submenuId = MENU_MONITOR;
      break;
    case KEY_SELECT:
      if (sub->varsNamed.nextAction == EDITACTION_ADD)
      {
        ++sub->varsNamed.stepsCount;
        if (sub->varsNamed.stepsCount > MAX_STEPS)
          sub->varsNamed.stepsCount = MAX_STEPS;
        else
        {
          menu->subMenus[menu->submenuId].stepId = sub->varsNamed.stepsCount-1;
          sub->varsNamed.subAction = EDITSUBACTION_TYPE;
          sub->varsNamed.executingAction = true;
        }
      }
      else if (sub->varsNamed.nextAction == EDITACTION_EDIT)
      {
        sub->varsNamed.executingAction = true;
      }
      else if (sub->varsNamed.nextAction == EDITACTION_REMOVE)
      {
        struct TemperatureStep * stepsArr = sub->varsNamed.steps;
        int i;
        sub->varsNamed.stepsCount = max(sub->varsNamed.stepsCount-1, 0);
        for (i=stepId; i < sub->varsNamed.stepsCount; ++i)
        {
          // move stuff
          memcpy(&stepsArr[i], &stepsArr[i+1], sizeof(TemperatureStep));
        }
        for (; i<MAX_STEPS; ++i)
        {
          memset(&stepsArr[i], 0, sizeof(TemperatureStep));
        }

        stepId = min(stepId, sub->varsNamed.stepsCount-1);
        menu->subMenus[menu->submenuId].stepId = stepId;
        activateStepsConfiguration(menu);
      }

      break;
    default:
      break;
    }
  }
  else if (sub->varsNamed.nextAction == EDITACTION_ADD ||
    sub->varsNamed.nextAction == EDITACTION_EDIT)
  {
    static int s_tempIncrement = 5, s_tempIncrementIdx = 0;
    static int s_tempIncrements[4] = { 
      5, 10, 50, 100             };

    struct TemperatureStep * curStep = &sub->varsNamed.steps[stepId];
    switch(buttonHit.key)
    {
    case KEY_UP:
      if (sub->varsNamed.subAction == EDITSUBACTION_TYPE)
        curStep->holdTemp = !curStep->holdTemp;
      else if (sub->varsNamed.subAction == EDITSUBACTION_TEMP)
      {
        if (buttonHit.action & ACTION_LONGCLICK && s_tempIncrementIdx < 3)
        {
          ++s_tempIncrementIdx;
          s_tempIncrement = s_tempIncrements[s_tempIncrementIdx];
        }
        curStep->targetTemp += s_tempIncrement;
      }
      else if (sub->varsNamed.subAction == EDITSUBACTION_DURATION)
        curStep->duration += 5;

      break;
    case KEY_DOWN:
      if (sub->varsNamed.subAction == EDITSUBACTION_TYPE)
        curStep->holdTemp = !curStep->holdTemp;
      else if (sub->varsNamed.subAction == EDITSUBACTION_TEMP)
      {
        if (buttonHit.action & ACTION_LONGCLICK && s_tempIncrementIdx > 0)
        {
          --s_tempIncrementIdx;
          s_tempIncrement = s_tempIncrements[s_tempIncrementIdx];
        }
        curStep->targetTemp -= s_tempIncrement;
      }
      else if (sub->varsNamed.subAction == EDITSUBACTION_DURATION)
        curStep->duration -= 5;

      curStep->duration = max(curStep->duration, 0);
      curStep->targetTemp = max(curStep->targetTemp, 0);

      break;
    case KEY_RIGHT:
      if (sub->varsNamed.nextAction == EDITACTION_EDIT)
      {
        // move to next step
        if (stepId < sub->varsNamed.stepsCount-1)
          ++menu->subMenus[menu->submenuId].stepId;
      }
      break;
    case KEY_LEFT:
      if (sub->varsNamed.nextAction == EDITACTION_EDIT)
      {
        // move to prev step
        if (stepId > 0)
          --menu->subMenus[menu->submenuId].stepId;
      }
      break;
    case KEY_SELECT:
      if (buttonHit.action & ACTION_LONGCLICK)
      {
        // save values and exit
        activateStepsConfiguration(menu);
        sub->varsNamed.executingAction = false;
      }
      else
      {
        ++sub->varsNamed.subAction;
        if (sub->varsNamed.subAction >2)
          sub->varsNamed.subAction = 0;
      }
      break;
    default:
      break;
    }

  }
}

void handleButtonsRunSteps(struct Menu * menu, struct ButtonHit buttonHit)
{
  switch(buttonHit.key)
  {
  case KEY_RIGHT:
    resetStepsRun(menu);
    menu->submenuId = MENU_MONITOR;
    break;
  case KEY_UP:
    break;
  case KEY_DOWN:
    break;
  case KEY_LEFT:
    resetStepsRun(menu);
    menu->submenuId = MENU_EDITSTEPS;
    break;
  case KEY_SELECT:
    break;
  default:
    break;
  }
}



void handleButtons(struct Menu * menu, struct ButtonHit buttonHit)
{
  if((buttonHit.action & ACTION_UP) == 0)
    return;

  switch(menu->submenuId)
  {
  case MENU_MONITOR:
    handleButtonsMonitor(menu, buttonHit);
    break;	
  case MENU_EDITSTEPS:
    handleButtonsEditSteps(menu, buttonHit);
    break;	
  case MENU_RUNSTEPS:
    handleButtonsRunSteps(menu, buttonHit);
    break;	
  }
}

void resetMenuMonitor(struct SubMenu *item)
{
  item->stepId = 0;
  // TODO: custom vars here
}

void resetMenuEditSteps(struct SubMenu *item)
{
  int i;

  item->stepId = 0;
  EditSubmenu * sub = (EditSubmenu*)item->vars;
  sub->varsNamed.stepsCount = 0;
  sub->varsNamed.nextAction = 0;
  sub->varsNamed.executingAction = false;

  for (i=0; i<MAX_STEPS; ++i)
  {
    sub->varsNamed.steps[i].targetTemp = 0;
    sub->varsNamed.steps[i].duration = 0;
    sub->varsNamed.steps[i].holdTemp = true;
  }
}

void resetMenuRunSteps(struct SubMenu *item)
{
  item->stepId = 0;
  // TODO: custom vars here
}



void resetMenuItem(struct Menu * menu, int item)
{
  switch(item)
  {
  case MENU_MONITOR:
    resetMenuMonitor(&menu->subMenus[item]);
    break;	
  case MENU_EDITSTEPS:
    resetMenuEditSteps(&menu->subMenus[item]);
    break;	
  case MENU_RUNSTEPS:
    resetMenuRunSteps(&menu->subMenus[item]);
    break;	
  }
}

void resetMenu(struct Menu * menu)
{
  int i;
  for(i=0; i<MENUITEMS_SIZE; ++i)
    resetMenuItem(menu, i);

  menu->submenuId = MENU_MONITOR;
}

/////      End menu logic


LiquidCrystal lcd( 8, 9, 4, 5, 6, 7 );
int ktcSO = 11;
int ktcCS = 12;
int ktcCLK = 13;
MAX6675 ktc(ktcCLK, ktcCS, ktcSO);


struct Menu menu;


void setup()
{ 
  Serial.begin(9600);
  initSSR();

  // give the MAX6557 a little time to settle
  delay(500);

  lcd.begin(16, 2);
  resetMenu(&menu);
  resetSteps(&s_configuredSteps[0]);
}

double readTemp()
{
  static double s_temp = 0;
  static int s_count = 0;

  if (s_count == 0)
    s_temp = ktc.readCelsius();

  ++s_count;
  if (s_count > 2)
    s_count = 0;

  return s_temp;
}

void updateTempAverage()
{
  tempWindow[nextSamplePos] = tempC;
  ++nextSamplePos;
  if (nextSamplePos >= AVG_TEMP_SAMPLES)
    nextSamplePos = 0;

  double tempSum = 0;
  for (int i=0; i<AVG_TEMP_SAMPLES; ++i)
    tempSum += tempWindow[i];

  tempAverage = tempSum / AVG_TEMP_SAMPLES;
}


void loop()
{
  // your main loop code here...
  s_msSinceStart = millis();

  struct ButtonHit btn = checkButton();
  char * line1;
  char * line2;

  tempC = readTemp();
  updateTempAverage();

  Serial.print("Deg C = "); 
  Serial.print(tempC);
  Serial.print("; avg T=");
  Serial.println(tempAverage);

  handleButtons(&menu, btn);
  getCurrentMessage(&menu, &line1, &line2);
  lcd.setCursor(0,0);
  lcd.print(line1);
  lcd.setCursor(0,1);
  lcd.print(line2);

  lcd.setCursor(10,1);
  //switch(btn.s_lastNonEmptyKey)
  switch(btn.key)
  {
  case KEY_RIGHT:
    lcd.print ("Right ");
    break;
  case KEY_UP:
    lcd.print ("Up    ");
    break;
  case KEY_DOWN:
    lcd.print ("Down  ");
    break;
  case KEY_LEFT:
    lcd.print ("Left  ");
    break;
  case KEY_SELECT:
    lcd.print ("Select");
    break;
  case KEY_RESET:
    lcd.print ("Reset ");
    break;
  case KEY_NONE:
    //lcd.print ("      ");
    break;

  }

  if (btn.key != KEY_NONE)
  {
    lcd.setCursor(9,1);
    //const char *clickStr = btn.s_lastNonEmptyAction == ACTION_NONE ? " " : (btn.s_lastNonEmptyAction == ACTION_DOWN ? "+" : "-");
    const char *clickStr = btn.action == ACTION_NONE ? " " : (btn.action == ACTION_DOWN ? "+" : "-");
    lcd.print(clickStr);
  }

  delay(100);
  ++s_ticks;
}









