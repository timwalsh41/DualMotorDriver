// Just increment the count by 1 ms each time
unsigned long micros()
{
    static unsigned long count = 0;

    count += 1000;
    return count;
}