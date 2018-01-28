nsorValue = sensorValue + analogRead(A2);
     delay(10);
   }
sensorValue=sensorValue/5;                          //Average of 5 readings
sensorValue = 1023 / sensorValue - 1;
  sensorValue = seriesResistor / sensorValue;
  steinhart = sensorValue / thermistorNominal;      // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= bCoefficient;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (temperatureNomina