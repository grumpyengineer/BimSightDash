#include "config.h"
#include "BMSModuleManager.h"

BMSModuleManager::BMSModuleManager()
{
  for (int i = 1; i <= MAX_MODULE_ADDR; i++) {
    modules[i].setExists(false);
    modules[i].setAddress(i);
  }
  lowestPackVolt = 1000.0f;
  highestPackVolt = 0.0f;
  lowestPackTemp = 200.0f;
  highestPackTemp = -100.0f;
  isFaulted = false;
}

bool BMSModuleManager::checkcomms()
{
  int g = 0;
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      g = 1;
      if (modules[y].isReset())
      {
        //Do nothing as the counter has been reset
      }
      else
      {
        return false;
      }
    }
    modules[y].setReset(false);
  }
  if ( g == 0)
  {
    return false;
  }
  return true;
}


bool BMSModuleManager::checkstatus()
{
  for (int y = 1; y < 20; y++)
  {
    if (modules[y].isExisting())
    {
      if ((modules[y].getError() & 0x2000) > 0)
      {
        return true;
      }
    }
  }
  return false;
}

int BMSModuleManager::seriescells()
{
  spack = 0;
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      spack = spack + modules[y].getscells();
    }
  }
  return spack;
}

void BMSModuleManager::clearmodules()
{
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      modules[y].clearmodule();
      modules[y].setExists(false);
      modules[y].setAddress(y);
    }
  }
}

int BMSModuleManager::getNumModules()
{
  return numFoundModules;
}

void BMSModuleManager::decodetemp(CAN_message_t &msg, int debug, int CSC)
{
  int CMU = (msg.id & 0x00F) + 1;
  modules[CMU].decodetemp(msg, CSC);
  if (debug == 1 && CMU > 0)
  {
    Serial.println();
    Serial.print(CMU);
    Serial.print(" Temp Found");
  }
}

void BMSModuleManager::decodecan(CAN_message_t &msg, int debug)
{
  int Id = (msg.id & 0x0F0);
  int CMU = (msg.id & 0x00F) + 1;
  /*
    if (msg.id == 0x100)
    {
    Serial.println(msg.id, HEX);
    }
  */
  switch (Id)
  {
    case 0x000:
      Id = 0;
      break;
    case 0x020:
      Id = 1;
      break;
    case 0x030:
      Id = 2;
      break;

    case 0x040:
      Id = 3;
      break;

    case 0x050:
      Id = 4;
      break;

    case 0x060:
      Id = 5;
      break;

    case 0x070:
      Id = 6;
      break;
  }
  if (CMU < 14 && Id < 7)
  {
    if (debug == 1)
    {
      Serial.print(CMU);
      Serial.print(",");
      Serial.print(Id);
      Serial.println();
    }
  }
  modules[CMU].setExists(true);
  modules[CMU].setReset(true);
  modules[CMU].decodecan(Id, msg, BalIgnore);
}

void BMSModuleManager::getAllVoltTemp()
{
  packVolt = 0.0f;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
#ifdef DEBUG
      Serial.println("");
      Serial.print("Module ");
      Serial.print(x);
      Serial.println(" exists. Reading voltage and temperature values");
      Serial.print("Module voltage: ");
      Serial.println(modules[x].getModuleVoltage());
      Serial.print("Lowest Cell V: ");
      Serial.print(modules[x].getLowCellV());
      Serial.print("     Highest Cell V:");
      Serial.println(modules[x].getHighCellV());
      Serial.print("Temp1: ");
      Serial.println(modules[x].getTemperature(0));
      Serial.print("       Temp2: ");
      Serial.println(modules[x].getTemperature(1));
#endif
      packVolt += modules[x].getModuleVoltage();
      if (modules[x].getLowTemp() < lowestPackTemp) lowestPackTemp = modules[x].getLowTemp();
      if (modules[x].getHighTemp() > highestPackTemp) highestPackTemp = modules[x].getHighTemp();
    }
  }

  packVolt = packVolt / Pstring;
  if (packVolt > highestPackVolt) highestPackVolt = packVolt;
  if (packVolt < lowestPackVolt) lowestPackVolt = packVolt;
#if 0
  if (digitalRead(11) == LOW) {
    if (!isFaulted) Serial.println("One or more BMS modules have entered the fault state!");
    isFaulted = true;
  }
  else
  {
    if (isFaulted) Serial.println("All modules have exited a faulted state");
    isFaulted = false;
  }
#endif
}

float BMSModuleManager::getLowCellVolt()
{
  LowCellVolt = 5.0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getLowCellV() <  LowCellVolt)  LowCellVolt = modules[x].getLowCellV();
    }
  }
  return LowCellVolt;
}

float BMSModuleManager::getHighCellVolt()
{
  HighCellVolt = 0.0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getHighCellV() >  HighCellVolt)  HighCellVolt = modules[x].getHighCellV();
    }
  }
  return HighCellVolt;
}

float BMSModuleManager::getPackVoltage()
{
  return packVolt;
}

float BMSModuleManager::getLowVoltage()
{
  return lowestPackVolt;
}

float BMSModuleManager::getHighVoltage()
{
  return highestPackVolt;
}

void BMSModuleManager::setBatteryID(int id)
{
  batteryID = id;
}

void BMSModuleManager::setBalIgnore(bool BalIgn)
{
  BalIgnore = BalIgn;
}

void BMSModuleManager::setPstrings(int Pstrings)
{
  Pstring = Pstrings;
}

void BMSModuleManager::setSensors(int sensor, float Ignore, int tempoff)
{
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      modules[x].settempsensor(sensor);
      modules[x].setIgnoreCell(Ignore);
      modules[x].setTempOff(tempoff);
    }
  }
}

float BMSModuleManager::getAvgTemperature()
{
  float avg = 0.0f;
  lowTemp = 999.0f;
  highTemp = -999.0f;
  int y = 0; //counter for modules below -70 (no sensors connected)
  numFoundModules = 0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      numFoundModules++;
      if (modules[x].getAvgTemp() > -70)
      {
        avg += modules[x].getAvgTemp();
        if (modules[x].getHighTemp() > highTemp)
        {
          highTemp = modules[x].getHighTemp();
        }
        if (modules[x].getLowTemp() < lowTemp)
        {
          lowTemp = modules[x].getLowTemp();

        }
      }
      else
      {
        y++;
      }
    }
  }
  avg = avg / (float)(numFoundModules - y);

  return avg;
}

float BMSModuleManager::getHighTemperature()
{
  return highTemp;
}

float BMSModuleManager::getLowTemperature()
{
  return lowTemp;
}

float BMSModuleManager::getAvgCellVolt()
{
  float avg = 0.0f;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting()) avg += modules[x].getAverageV();
  }
  avg = avg / (float)numFoundModules;

  return avg;
}

void BMSModuleManager::printPackSummary()
{
  uint8_t faults;
  uint8_t alerts;
  uint8_t COV;
  uint8_t CUV;

  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.print("Modules: ");
  Serial.print(numFoundModules);
  Serial.print("  Cells: ");
  Serial.print(seriescells());
  Serial.print("  Voltage: ");
  Serial.print(getPackVoltage());
  Serial.print("V   Avg Cell Voltage: ");
  Serial.print(getAvgCellVolt());
  Serial.print("V     Avg Temp: ");
  Serial.print(getAvgTemperature());
  Serial.println("C");

  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      faults = modules[y].getFaults();
      alerts = modules[y].getAlerts();
      COV = modules[y].getCOVCells();
      CUV = modules[y].getCUVCells();

      Serial.print("                               Module #");
      Serial.println(y);

      Serial.print("  Voltage: ");
      Serial.print(modules[y].getModuleVoltage());
      Serial.print("V   (");
      Serial.print(modules[y].getLowCellV());
      Serial.print("V-");
      Serial.print(modules[y].getHighCellV());
      Serial.print("V)     Temperatures: (");
      Serial.print(modules[y].getLowTemp());
      Serial.print("C-");
      Serial.print(modules[y].getHighTemp());
      Serial.println("C)");

      if (faults > 0)
      {
        Serial.println("  MODULE IS FAULTED:");
        if (faults & 1)
        {
          Serial.print("    Overvoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 12; i++)
          {
            if (COV & (1 << i))
            {
              Serial.print(i + 1);
              Serial.print(" ");
            }
          }
          Serial.println();
        }
        if (faults & 2)
        {
          Serial.print("    Undervoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 12; i++)
          {
            if (CUV & (1 << i))
            {
              Serial.print(i + 1);
              Serial.print(" ");
            }
          }
          Serial.println();
        }
        if (faults & 4)
        {
          Serial.println("    CRC error in received packet");
        }
        if (faults & 8)
        {
          Serial.println("    Power on reset has occurred");
        }
        if (faults & 0x10)
        {
          Serial.println("    Test fault active");
        }
        if (faults & 0x20)
        {
          Serial.println("    Internal registers inconsistent");
        }
      }
      if (alerts > 0)
      {
        Serial.println("  MODULE HAS ALERTS:");
        if (alerts & 1)
        {
          Serial.println("    Over temperature on TS1");
        }
        if (alerts & 2)
        {
          Serial.println("    Over temperature on TS2");
        }
        if (alerts & 4)
        {
          Serial.println("    Sleep mode active");
        }
        if (alerts & 8)
        {
          Serial.println("    Thermal shutdown active");
        }
        if (alerts & 0x10)
        {
          Serial.println("    Test Alert");
        }
        if (alerts & 0x20)
        {
          Serial.println("    OTP EPROM Uncorrectable Error");
        }
        if (alerts & 0x40)
        {
          Serial.println("    GROUP3 Regs Invalid");
        }
        if (alerts & 0x80)
        {
          Serial.println("    Address not registered");
        }
      }
      if (faults > 0 || alerts > 0) Serial.println();
    }
  }
}

void BMSModuleManager::printPackDetails(int digits, int CSCvariant)
{
  uint8_t faults;
  uint8_t alerts;
  uint8_t COV;
  uint8_t CUV;
  int cellNum = 0;

  Serial.println("");
  Serial.print("Modules: ");
  Serial.print(numFoundModules);
  Serial.print(" Cells: ");
  Serial.print(seriescells());
  Serial.print(" Strings: ");
  Serial.print(Pstring);
  Serial.print("  Voltage: ");
  Serial.print(getPackVoltage(), digits);
  Serial.println("V");
  Serial.print("Avg Cell Voltage: ");
  Serial.print(getAvgCellVolt(), digits);
  Serial.print("V  Low Cell Voltage: ");
  Serial.print(getLowCellVolt(), digits);
  Serial.print("V   High Cell Voltage: ");
  Serial.print(getHighCellVolt(), digits);
  Serial.print("V Delta Voltage: ");
  Serial.print((getHighCellVolt() - getLowCellVolt()) * 1000);
  Serial.print("mV   Avg Temp: ");
  Serial.print(getAvgTemperature());
  Serial.println("C ");
  Serial.println("");
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      faults = modules[y].getFaults();
      alerts = modules[y].getAlerts();
      COV = modules[y].getCOVCells();
      CUV = modules[y].getCUVCells();

      Serial.print("Module ");
      Serial.print(y);
      Serial.print('\t');
      Serial.print(modules[y].getModuleVoltage(), digits);
      Serial.println("V");
      for (int i = 0; i < 16; i++)
      {
        Serial.print("Cell ");
        Serial.print(cellNum++);
        Serial.print(": ");
        if(cellNum <= 10)
          Serial.print(" ");
        Serial.print(modules[y].getCellVoltage(i), digits);
        Serial.print("V");
        Serial.print('\t');
        if(i == 7)
          Serial.println();
      }

      Serial.println();

      Serial.print("Temp 1: ");
      Serial.print(modules[y].getTemperature(0));
      Serial.print("C");
      Serial.print('\t');
      Serial.print("Temp 2: ");
      Serial.print(modules[y].getTemperature(1));
      Serial.print("C");
      Serial.print('\t');
      Serial.print("Status: 0x");
      Serial.print(modules[y].getError(), HEX);
      Serial.print('\t');
      Serial.print("Bal: 0x");
      Serial.println(modules[y].getbalstat(), HEX);

    }
  }
}
