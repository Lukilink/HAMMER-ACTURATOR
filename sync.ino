
/* When you run this sketch, you will see 3 diffrent rows in serial monitor.
 * the values in the second row are always a view milliseconds behind the first row.
 * The goal is, to add a delay to value and store it in "value_with_added_delay". 
 * So that the value value_with_added_delay is synced to value_with_delay.
 */


unsigned long Time;
unsigned long value = 10;
unsigned long value_with_delay;
unsigned long value_with_added_delay;



void setup() {
Serial.begin(9600);

}

void loop() {
value += 1; 
value_with_delay  +=1;

             
Serial.print(value);
Serial.print("  ");
Serial.print(value_with_delay);
Serial.print("  ");
Serial.print(value_with_added_delay);
Serial.print("  ");
Serial.println("");
  

}
