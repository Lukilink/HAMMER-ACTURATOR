
/* When you run this sketch, you will see 3 diffrent column in serial monitor.
 * the values in the second column are always a few milliseconds behind the values in first column.
 * The goal is, to add a delay to value and store it in "value_with_added_delay". 
 * So that the value_with_added_delay is showing the same like value_with_delay.
 */


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
