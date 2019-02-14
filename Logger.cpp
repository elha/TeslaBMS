/*
 * Logger.cpp
 *
  Copyright (c) 2013 Collin Kidder, Michael Neuweiler, Charles Galpin

 Permission is hereby granted, free of charge, to any person obtaining
 a copy of this software and associated documentation files (the
 "Software"), to deal in the Software without restriction, including
 without limitation the rights to use, copy, modify, merge, publish,
 distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so, subject to
 the following conditions:

 The above copyright notice and this permission notice shall be included
 in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 
  */

#include "Logger.h"
#include <stdarg.h>

#define ARDBUFFER 8

Logger::LogLevel Logger::logLevel = Logger::Info;
uint32_t Logger::lastLogTime = 0;

/*
 * Output a debug message with a variable amount of parameters.
 * printf() style, see Logger::log()
 *
 */
void Logger::debug(char *message, ...)
{
    if (logLevel > Debug)
        return;
    va_list args;
    va_start(args, countParams(message));
    Logger::log(Debug, message, args);
    va_end(args);
}

/*
 * Output a info message with a variable amount of parameters
 * printf() style, see Logger::log()
 */
void Logger::info(char *message, ...)
{
    if (logLevel > Info)
        return;
    va_list args;
    va_start(args, countParams(message));
    Logger::log(Info, message, args);
    va_end(args);
}

/*
 * Output a warning message with a variable amount of parameters
 * printf() style, see Logger::log()
 */
void Logger::warn(char *message, ...)
{
    if (logLevel > Warn)
        return;
    va_list args;
    va_start(args, countParams(message));
    Logger::log(Warn, message, args);
    va_end(args);
}

/*
 * Output a error message with a variable amount of parameters
 * printf() style, see Logger::log()
 */
void Logger::error(char *message, ...)
{
    if (logLevel > Error)
        return;
    va_list args;
    va_start(args, countParams(message));
    Logger::log(Error, message, args);
    va_end(args);
}

/*
 * Output a comnsole message with a variable amount of parameters
 * printf() style, see Logger::ardprintf()
 */
void Logger::console(char *message, ...)
{
    va_list args;
    va_start(args, countParams(message));
    Logger::ardprintf(message, args);
    va_end(args);
}

/*
 * Set the log level. Any output below the specified log level will be omitted.
 */
void Logger::setLoglevel(LogLevel level)
{
    logLevel = level;
}

/*
 * Retrieve the current log level.
 */
Logger::LogLevel Logger::getLogLevel()
{
    return logLevel;
}

/*
 * Return a timestamp when the last log entry was made.
 */
uint32_t Logger::getLastLogTime()
{
    return lastLogTime;
}

/*
 * Returns if debug log level is enabled. This can be used in time critical
 * situations to prevent unnecessary string concatenation (if the message won't
 * be logged in the end).
 *
 * Example:
 * if (Logger::isDebug()) {
 *    Logger::debug("current time: %d", millis());
 * }
 */
boolean Logger::isDebug()
{
    return logLevel == Debug;
}

/*
 * Output a log message (called by debug(), info(), warn(), error(), console())
 *
 * Supports printf() like syntax:
 *
 * %% - outputs a '%' character
 * %s - prints the next parameter as string
 * %d - prints the next parameter as decimal
 * %f - prints the next parameter as double float
 * %x - prints the next parameter as hex value
 * %X - prints the next parameter as hex value with '0x' added before
 * %b - prints the next parameter as binary value
 * %B - prints the next parameter as binary value with '0b' added before
 * %l - prints the next parameter as long
 * %c - prints the next parameter as a character
 * %t - prints the next parameter as boolean ('T' or 'F')
 * %T - prints the next parameter as boolean ('true' or 'false')
 */
void Logger::log(LogLevel level, char *format, va_list argv)
{
    lastLogTime = millis();

    Logger::ardprintf(format, argv);
}

/*
 * Output a log message (called by log(), console())
 *
 * ardprintf by https://gist.github.com/asheeshr/9004783
 * Supports printf() like syntax:
 *
 * %% - outputs a '%' character
 * %s - prints the next parameter as string
 * %d - prints the next parameter as decimal
 * %f - prints the next parameter as double float
 * %x - prints the next parameter as hex value
 * %X - prints the next parameter as hex value with '0x' added before
 * %b - prints the next parameter as binary value
 * %B - prints the next parameter as binary value with '0b' added before
 * %l - prints the next parameter as long
 * %c - prints the next parameter as a character
 * %t - prints the next parameter as boolean ('T' or 'F')
 * %T - prints the next parameter as boolean ('true' or 'false')
 */
void Logger::ardprintf(char *str, va_list argv)
{
  int i, j=0, flag=0;
  char temp[ARDBUFFER+1];

  for(i=0,j=0; str[i]!='\0';i++) //Iterate over formatting string
  {
    if(str[i]=='%')
    {
      //Clear buffer
      temp[j] = '\0'; 
      SERIALCONSOLE.print(temp);
      j=0;
      temp[0] = '\0';
      
      //Process argument
      switch(str[++i])
      {
        case 'd': SERIALCONSOLE.print(va_arg(argv, int));
                  break;
        case 'i': SERIALCONSOLE.print(va_arg(argv, int));
                  break;
        case 'l': SERIALCONSOLE.print(va_arg(argv, long), DEC);
                  break;
        case 'f': SERIALCONSOLE.print(va_arg(argv, double), 3);
                  break;
        case 'c': SERIALCONSOLE.print((char)va_arg(argv, int));
                  break;
        case 's': SERIALCONSOLE.print(va_arg(argv, char *));
                  break;
        case 'X': SERIALCONSOLE.print("0x");
        case 'x': SERIALCONSOLE.print(va_arg(argv, int), HEX);
                  break;
        case 'B': SERIALCONSOLE.print("0b");
        case 'b': SERIALCONSOLE.print(va_arg(argv, int), BIN);
                  break;
        case 't': 
                if (va_arg(argv, int) == 1)
                {
                    SERIALCONSOLE.print("T");
                }
                else
                {
                    SERIALCONSOLE.print("F");
                }
                break;
        case 'T': 
                if (va_arg(argv, int) == 1)
                {
                    SERIALCONSOLE.print("TRUE");
                }
                else
                {
                    SERIALCONSOLE.print("FALSE");
                }
                break;
        default:  ;
      };
    }
    else 
    {
      //Add to buffer
      temp[j] = str[i];
      j = (j+1)%ARDBUFFER;
      if(j==0)  //If buffer is full, empty buffer.
      {
        temp[ARDBUFFER] = '\0';
        SERIALCONSOLE.print(temp);
        temp[0]='\0';
      }
    }
  };
  
  if(j>0) {
     temp[j] = '\0'; 
     SERIALCONSOLE.print(temp);
  }
  
  SERIALCONSOLE.println(); //Print trailing newline
}

int countParams(char *str) {
    int count = 0;
    for(int i=0; str[i]!='\0';i++)  if(str[i]=='%')  count++; //Evaluate number of arguments required to be printed
    return count;
}
