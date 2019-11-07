/*Non-Canonical Input Processing*/


#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <math.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS1"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define FLAG 0x7e
#define A 0x03
#define UA 0x07
#define RR 0x05
#define DISC 0x0b
#define REJ 0x01
#define MAXOFFSET 5


/**
 * struct usada para guardar dados das tramas
 * em caso de erro n_flags = -1
 */
typedef struct
{
  int cc, n_sequencia, n_octetos;
  unsigned char bcc;
  unsigned char informacao[255];
} Dados;

typedef struct
{
  size_t n_flags;
  bool dados;
  unsigned char a, cs, bcc;
  Dados informacao;
  unsigned char *flags;
} Trama;

float T_PROP = 0.0; //tempo de propagacao em segundos
int FER_percentage = 0; //probabilidade de haver erro artificial (n inteiro)
int T_trama = 260; //tamanho da trama de dados -> 5 + tamanho do campo de informacacao (default 5 + 255 = 260)

int bytesSent = 0;
int fileSize = 0;
int transferedSize = 0;
char FICHEIRO[255] = "./pinguim.gif";
volatile int STOP = FALSE;
bool alarmStop = FALSE;
bool emitter;

void clear(){
    #if defined(__linux__) || defined(__unix__) || defined(__APPLE__)
        system("clear");
    #endif

    #if defined(_WIN32) || defined(_WIN64)
        system("cls");
    #endif
}

void atende()
{
  alarmStop = TRUE;
  printf("Timeout\n");
}
void activateAlarm(int s)
{
  alarmStop = FALSE;
  alarm(s);
}

void printTrama(Trama * t){
  printf("\n*********************\n");
  if(t->n_flags == -1) {printf("Erro: nFlags = -1\n"); printf("\n*********************\n"); return;}

  for(int i = 0; i < t->n_flags; i++){
    printf("Flag %d -> %x\n", i+1, t->flags[i]);
  }

  printf("Addr -> %x\n", t->a);
  printf("C_set -> %x\n", t->cs);
  printf("Bcc -> %x\n", t->bcc);

  printf("Dados -> %d\n", t->dados);

  if(t->dados){
    printf("****** DADOS ******\n");
    printf("C -> %d\n", t->informacao.cc);

    if(t->informacao.cc == 1){
      printf("N -> %d\n", t->informacao.n_sequencia);
      printf("Ls -> %d\n", t->informacao.n_octetos);
      for(int a = 0; a < t->informacao.n_octetos; a++){
        printf("%x ", t->informacao.informacao[a]);
      }
      printf("\nBcc2 -> %x\n", t->informacao.bcc);
    }

  }
  printf("\n*********************\n");
}

// retorna 1 em caso de erro
int sendTrama(Trama *t, int fd)
{
  /**
  * parse and write
  */

  int tamanho = t->n_flags * 2 + 3 + (t->dados ? 5 + t->informacao.n_octetos : 0);
  unsigned char *set = (unsigned char *)calloc(tamanho, sizeof(unsigned char));

  int next = 0;

  while(next < t->n_flags){
    *(set + next) = *(t->flags + next);
    next++;
  }

  set[next++] = t->a;
  set[next++] = t->cs;
  set[next++] = t->bcc;


  if(t->dados){
    unsigned char xor = 0x00;

    *(set + next++) = t->informacao.cc;
    xor ^= t->informacao.cc;

    if(t->informacao.cc == 1){
      *(set + next++) = t->informacao.n_sequencia;
      xor ^= t->informacao.n_sequencia;
      *(set + next++) = t->informacao.n_octetos / 256;
      xor ^= t->informacao.n_octetos / 256;
      *(set + next++) = t->informacao.n_octetos % 256;
      xor ^= t->informacao.n_octetos % 256;

      int stop = next + t->informacao.n_octetos;
      for (int f = 0; next < stop; f++){
        xor ^= *(t->informacao.informacao + f);
        *(set + next++) = *(t->informacao.informacao + f);
      }
    }
    else{
      *(set + next++) = 0;
      xor ^= 0;

      int nb = (int)(log(fileSize)/log(256)) + 1;
      *(set + next++) = nb;
      xor ^= (int)(log(fileSize)/log(256)) + 1;

      for(int i = 1; i <= nb; i++){
        unsigned char byte = fileSize / pow(256, nb - i);
        *(set + next++) = byte;
        xor ^= byte;
      }

    }

    *(set + next++) = xor;
  }

  int stop = next + t->n_flags;
  for (int f = 0; next < stop; f++)
  {
    *(set + next++) = *(t->flags + f);

  }

  /*printf("\n\n____SET____\n");
  for(int i = 0; i < next; i++){
    printf("%x ", set[i]);
  }
  printf("\n__________\n\n");*/

  int res = write(fd, set, next);
  bytesSent += res;
  nanosleep((const struct timespec[]){{0, T_PROP * 1000000000L}}, NULL);
  //printf("%d bytes written\n", res);
  free(set);

  if (res < tamanho)
  {
    return 1;
  }
  return 0;
}

Trama *fillConnectionTrama(unsigned char cset)
{
  Trama *t = calloc(1, sizeof(Trama));

  t->n_flags = 1;

  t->dados = false;

  t->flags = (unsigned char *)calloc(t->n_flags, sizeof(unsigned char));

  t->flags[0] = FLAG;

  t->a = A;

  t->cs = cset;

  t->bcc = t->a ^ t->cs;

  return t;
}

Trama *fillDataTrama(int c, int n, int tam, unsigned char *dados){
  Trama * t = fillConnectionTrama(UA);

  t->dados = true;
  t->informacao.cc = c;
  t->informacao.n_sequencia = n;
  t->informacao.n_octetos = tam;
  memcpy(t->informacao.informacao, dados, tam);

  return t;
}

Trama *sendConnectionTrama(int fd, unsigned char c)
{

  Trama *t = fillConnectionTrama(c);

  if (sendTrama(t, fd))
  {
    t->n_flags = -1;
  }

  return t;
}

Trama *receiveData(int fd, bool alarm)
{
  unsigned char received_byte;
  unsigned char received_array[400];

  Trama *t = calloc(1, sizeof(Trama));

  if (alarm)
    (void)signal(SIGALRM, atende);

  int state = 0, res;
  int flag_count = -1, inf_count = 0;
  bool tramaI = true;
  bool ERRO = false;
  bool EXIT = FALSE;
  bool alarmSet = FALSE;
  bool controlo = false;
  unsigned char * dados = calloc(260, sizeof(unsigned char));
  unsigned char xor = 0x00;

  //printf("Waiting for response\n");

  for (int i = 0; state < 5 && !EXIT && !ERRO;){

    res = read(fd, &received_byte, 1);
    if (res == 0){

      if (alarm){
        if (!alarmSet){
          activateAlarm(3);
          alarmSet = true;
        }

        if (alarmStop){
          break;
        }
      }

      continue;
    }

    //printf("Byte : %x  at state : %d with inf_count at %d\n", received_byte, state, inf_count);

    if (alarm){
      if (alarmSet){
        activateAlarm(0);
        alarmSet = false;
      }
    }

    received_array[i] = received_byte;
    switch (state){
    case 0:
      flag_count++;
      if (received_byte == A)
        state++;
      break;
    case 1:
      state++;
      break;
    case 2:
      state++;
      if (received_byte != (received_array[flag_count] ^ received_array[flag_count + 1]))
      {
        printf("Erro byte C\n");

        EXIT = TRUE;
        ERRO = true;
      }
      break;
    case 3:
      if ((flag_count == 0 ? received_byte == FLAG : received_byte == received_array[0]) && inf_count == 0)
      {
        // trama de supervisao ou nao numerada
        tramaI = false;
        inf_count = -1;
        EXIT = TRUE;
      }
      else{
        // trama de informacao
        if(inf_count > 260){
          ERRO = true;
          break;
        }
        if(inf_count == 3){
          if(dados[0] != 1){
            controlo = true;
          }
        }
        if(!controlo){
          if(inf_count > 3){
            if((int)dados[2] * 256 + (int)dados[3] == inf_count - 4){
              if(received_byte == xor){
                state++;
              }
              else{
                ERRO = true;
                EXIT = true;
              }
              break;
            }
          }
        }
        else{
          if(inf_count == dados[2] + 3){
            if(received_byte == xor){
              state++;
            }
            else{
              ERRO = true;
            }
            break;
          }
        }

        dados[inf_count++] = received_byte;
        xor ^= received_byte;
      }

      break;
    case 4:
      // confirmacao da trama de informacao
      if(received_byte == received_array[0] || received_byte == FLAG){
        EXIT = true;
        //printf("Erro na trama");
      }
      else{
        ERRO = true;
        /*
        dados[inf_count] = received_byte;
        xor ^= received_byte;
        inf_count++;
        state--;*/
      }
      break;
    }
    i++;
  }


  if (!ERRO && !alarmStop)
  {
    /*for(int i = 0; i < 2 * flag_count + 3; i++){
      printf("%x\n", received_array[i]);
    }*/

    // passa received array para trama
    t->flags = calloc(flag_count, sizeof(unsigned char));
    for (int i = 0; i < flag_count; i++)
    {
      t->flags[i] = received_array[i];
    }
    t->n_flags = flag_count;
    t->dados = (inf_count == -1 ? false : true);
    if(tramaI){
      t->informacao.cc = dados[0];
      if(dados[0] == 1){
        t->informacao.n_sequencia = dados[1];
        t->informacao.n_octetos = 256*dados[2]+dados[3];
        for(int j = 0; j < t->informacao.n_octetos; j++){
          t->informacao.informacao[j] = dados[4+j];
        }
        t->informacao.bcc = xor;
      }
      else{
        if(dados[0] == 2){
          if(dados[1] == 0){
            int fs = 0;
            for(int i = dados[2] - 1, a = 0; i >= 0; i--, a++){
              fs += pow(256, i) * dados[3 + a];
            }
            fileSize = fs;
          }
        }
      }
    }
    t->a = received_array[flag_count];
    t->cs = received_array[flag_count + 1];
    t->bcc = received_array[flag_count + 2];
  }
  else
  {
    t->n_flags = -1;
  }

  if (alarmStop)
    printf("Erro: timed out\n");

  alarmStop = FALSE;

  return t;
}

int connect(int fd)
{
  //***************** ligacao inicial *******************

  clear();
  printf("Connecting\n");
  int alarmCnt = 0;
  bool erro = false;

  for (alarmCnt = 0; alarmCnt < 3; alarmCnt++)
  {

    Trama *sent;
    Trama *received;

    if (emitter)
    {
      sent = sendConnectionTrama(fd, A);
      if (sent->n_flags == -1)
      {
        printf("Erro a enviar trama\n");
        erro = true;
        break;
      }
    }
    else
    {
      received = receiveData(fd, emitter);

      if (received->n_flags == -1)
      {
        printf("Erro a enviar trama\n");
        erro = true;
        break;
      }
    }



    if (emitter)
    {
      received = receiveData(fd, emitter);

      if (received->n_flags == -1)
      {
        printf("\nErro a receber trama\nTentando novamente\n\n");
      }
      else
      {
        break;
      }
    }
    else
    {
      sent = sendConnectionTrama(fd, UA);
      if (sent->n_flags == -1)
      {
        printf("\nErro a enviar trama\n\n");
        erro = true;
      }
      break;
    }
  }

  if (alarmCnt == 3 || erro)
  {
    printf("Demasiados erros, saindo\n\n");
    return 1;
  }
  else
  {
    printf("Connected\n\n");
    return 0;
  }
  //*******************************
}

FILE * openFile(){
  FILE *fp;

  if (emitter)
    fp = fopen(FICHEIRO, "rb");
  else
    fp = fopen(FICHEIRO, "wb");

  return fp;
}

int stuffFile(FILE * fp, unsigned char *** stuffedFile){
  (*stuffedFile) = calloc(fileSize * 2, sizeof(unsigned char *));
  (*stuffedFile)[0] = calloc(255, sizeof(unsigned char));

  //int cnt = 0;

  //missing calloc?
  int x = 1;
  int y = 0;

  //int rawByteSize = 0;

  while(1) {
    //printf("curr bit: %d    :::::    max size: %d\n", ++cnt, fileSize);
    unsigned char b = 0x00;

    fread(&b, 1, 1, fp);
    if(feof(fp)) break;
    //rawByteSize++;

    int t;

    switch (b)
    {
    case 0x7e:
      t = 2;

      if(x + t > T_trama - 6){
        //printf("Raw byte size at index %d: %d\n", y, rawByteSize);
        (*stuffedFile)[y][0] = x;
        (*stuffedFile)[y++][x] = 0x7d;
        (*stuffedFile)[y] = calloc(255, sizeof(unsigned char));
        x = 1;
      }

      (*stuffedFile)[y][x++] = 0x7d;
      (*stuffedFile)[y][x++] = 0x5e;

      break;

    case 0x7d:
      t = 2;

      if(x + t > T_trama - 6){
        //printf("Raw byte size at index %d: %d\n", y, rawByteSize);
        (*stuffedFile)[y][0] = x;
        (*stuffedFile)[y++][x] = 0x7d;
        (*stuffedFile)[y] = calloc(255, sizeof(unsigned char));
        x = 1;
      }

      (*stuffedFile)[y][x++] = 0x7d;
      (*stuffedFile)[y][x++] = 0x5d;

      break;

    default:
      t = 1;

      if(x + t > T_trama - 6){
        //printf("Raw byte size at index %d: %d\n", y, rawByteSize);
        (*stuffedFile)[y][0] = x;
        (*stuffedFile)[y++][x] = 0x7d;
        (*stuffedFile)[y] = calloc(255, sizeof(unsigned char));
        x = 1;
      }

      (*stuffedFile)[y][x++] = b;

      break;
    }
  }

  //printf("Raw byte size at index %d: %d\n", y, rawByteSize);

  if(x!=1){
    (*stuffedFile)[y][x] = 0x7d;
    (*stuffedFile)[y++][0] = x;
  }



  /*for(int i = 0; i < y; i++){
      printf("n bytes em trama indice %d -> %d :  ", i, (*stuffedFile)[i][0]);
      for(int a  = 1; a < 10 ; a++) printf("%x:", (*stuffedFile)[i][a]);
      printf("\n");
  }*/

  return y;

}

/**
 *  v é o apontador para onde está a ser guardado
 * um vetor com os bytes do ficheiro após destuffing
 *  curIndice é o tamanho ocupado do ficheiro
 *  tam é o tamanho atual do vetor v
 *  d é o pacote de dados com informacão a adicionar a v
 */
void destuff(unsigned char ** v, int *curIndice, int *tam, Dados d){

  unsigned char * buf = calloc(d.n_octetos * 2, sizeof(unsigned char));

  int bufSize = 0;

  for(int i = 0; i < d.n_octetos; i++){
    switch(d.informacao[i]){
      case 0x7d:
        if(i+1 == d.n_octetos) break;
        else if(d.informacao[i+1] == 0x5e){
          buf[bufSize++] = 0x7e;
          i++;
        }
        else if(d.informacao[i+1] == 0x5d){
          buf[bufSize++] = 0x7d;
          i++;
        }
        break;

      default:
        buf[bufSize++] = d.informacao[i];
        break;
    }
  }


  if(*curIndice + bufSize + 1 > *tam){
    *v = realloc(*v, *curIndice + 1 + bufSize);
    *tam = *curIndice + 1 + bufSize;
  }

  memcpy(*v + *curIndice, buf, bufSize);
  *curIndice = *curIndice + bufSize;


  free(buf);
}

int storeData(FILE ** fp, unsigned char * dados, int tamanho){
  return fwrite(dados, sizeof(unsigned char),tamanho,*fp);
}

int transfer_file(int fd)
{
  clear();

  printf("Preparing file\n");

  int n_tramas_dados = 0;
  FILE * fp;
  unsigned char ** stuffedFile;

  if(emitter){
    fp = openFile();

    if (fp == NULL) {
      printf("Erro a abrir ficheiro\n");
      return 1;
    }

    fseek(fp, 0L, SEEK_END);
    fileSize = ftell(fp);
    fseek(fp, 0L, SEEK_SET);

    n_tramas_dados = stuffFile(fp, &stuffedFile);
  }
  else{
    fp = openFile();
    if (fp == NULL) {
      printf("Erro a criar ficheiro\n");
      return 1;
    }
  }


  printf("Transfering\n");



  int counter = 0;
  int controlCnt = 0;

  int offset = 0;
  int lastR = 1;

  while(emitter ? (counter <= n_tramas_dados+1) : (controlCnt < 2 ? 1 : 0)){

    int externCounter;
    int alarmCnt = 0;
    bool erro = false;
    bool sucesso = false;

    Trama *sent;
    Trama *received;

    for (alarmCnt = 0; alarmCnt < 3 && !sucesso; alarmCnt++)
    {

      if (emitter)
      {
        if(counter % (n_tramas_dados+1) == 0){
          sent = fillDataTrama(2 + counter / (n_tramas_dados+1), 0, 0, NULL);
        }
        else{
          sent = fillDataTrama(1, counter%255, (int)stuffedFile[counter-1][0], stuffedFile[counter-1]+1);
        }

        int b = sendTrama(sent, fd);

        //printf("Sent package: %d\n", counter);
        //printTrama(sent);

        if (sent->n_flags == -1 || b)
        {
          printf("Erro a enviar trama\n");
          /*erro = true;
          break;*/
        }
      }
      else
      {
        received = receiveData(fd, emitter);

        //externCounter = received->informacao.n_sequencia;
        //printf("Received trama n: %d extern: %d\n", counter, externCounter);
        //printf("Current transferedSize: %d\n", transferedSize);
        //printTrama(received);

        if (received->n_flags == -1)
        {
          printf("Recebida trama com erros\n");
          erro = true;
          //break;
        }
      }


      /**********************************************/


      if (emitter)
      {
        received = receiveData(fd, 1);

        //printf("received:\n");
        //printTrama(received);

        if (received->n_flags == -1)
        {
          printf("\nErro a receber trama\nTentando novamente\n\n");
          free(sent);
          free(received);
        }
        else
        {
          break;
        }
      }
      else
      {
        unsigned char c;
        if(!erro)
          if(rand() % 99 < FER_percentage) c = REJ;
          else c = RR;
        else
          c = REJ;

        c |= (counter % 2 == 0 ? 0x00 : 0x80);
        sent = fillConnectionTrama(c);

        int b = sendTrama(sent, fd);

        //printf("sent: \n");
        //printTrama(sent);

        if (sent->n_flags == -1 || b)
        {
          printf("\nErro a enviar trama\n\n");
          erro = true;
        }
        break;
      }

    }

    if (alarmCnt == 3)
    {
      printf("Demasiados Timeouts, abortando\n\n");
      return 1;
    }
    else
    {
      //processar
      if(emitter){
        if(offset > MAXOFFSET){
          counter -= MAXOFFSET+3;
          offset = 0;
          printf("counter -3 -> %d\n", counter);
        }

        if(lastR == received->cs >> 7){
          offset++;
        }
        else offset = 0;

        unsigned char c = 0x7f;
        c &= received->cs;

        if (c == RR && !erro){
          //printf("Sent\n\n");
          sucesso = true;
          counter++;
        }
        else{
          printf("Reenviando pacote %d\n", counter);
          //printf("Received C: %x  --  Erro: %d\n", c, erro);
        }

        lastR = received->cs >> 7;
      }
      else{
        if(!erro){
          externCounter = received->informacao.n_sequencia;
          if(received->informacao.cc == 1 && externCounter == counter%255){
            int bufSize = 255, index = 0;
            unsigned char *buf = calloc(bufSize, sizeof(unsigned char));
            destuff(&buf, &index, &bufSize, received->informacao);
            transferedSize += index;
            storeData(&fp, buf, index);
            free(buf);
            counter++;
          }
          else if(received->informacao.cc == 2 || received->informacao.cc == 3){
            controlCnt++;
            counter++;
          }
        }
        free(received);
        free(sent);
      }

    }

    //prirnt to the console
    clear();
    printf("Transfering");
    for(int i = 0; i < counter % 4; i++) printf(".");
    if(emitter)
      printf("\n%.1f%%\n", 100.0*counter / (n_tramas_dados + 1.0));
    else
      printf("\n%.1f%%\n", 100.0*transferedSize / fileSize);

  }



  fclose(fp);
  return 0;
}

int ending(int fd){

  //***************** ligacao inicial *******************

  clear();
  printf("Disconnecting\n");
  int alarmCnt = 0;
  bool erro = false;

  for (alarmCnt = 0; alarmCnt < 3; alarmCnt++)
  {

    Trama *sent;
    Trama *received;

    if (emitter)
    {
      sent = sendConnectionTrama(fd, DISC);
      if (sent->n_flags == -1)
      {
        printf("Erro a enviar trama\n");
        erro = true;
        break;
      }
    }
    else
    {
      received = receiveData(fd, emitter);

      if (received->n_flags == -1)
      {
        printf("Erro a enviar trama\n");
        erro = true;
        break;
      }
    }



    if (emitter)
    {
      received = receiveData(fd, emitter);

      if (received->n_flags == -1)
      {
        printf("\nErro a receber trama\nTentando novamente\n\n");
      }
      else
      {
        break;
      }
    }
    else
    {
      sent = sendConnectionTrama(fd, DISC);
      if (sent->n_flags == -1)
      {
        printf("\nErro a enviar trama\n\n");
        erro = true;
      }
      break;
    }


    if (emitter)
    {
      sent = sendConnectionTrama(fd, UA);
      if (sent->n_flags == -1)
      {
        printf("Erro a enviar trama\n");
        erro = true;
        break;
      }
    }
    else
    {
      received = receiveData(fd, emitter);

      if (received->n_flags == -1)
      {
        printf("Erro a enviar trama\n");
        erro = true;
        break;
      }
    }

  }

  if (alarmCnt == 3 || erro)
  {
    printf("Demasiados erros (no ending), saindo\n\n");
    return 1;
  }
  else
  {
    printf("Done\n\n");
    return 0;
  }
  //*******************************

}

int main(int argc, char **argv)
{
  int fd;
  struct termios oldtio, newtio;

  if ((argc < 3) ||
      ((strcmp("/dev/ttyS0", argv[1]) != 0) &&
       (strcmp("/dev/ttyS1", argv[1]) != 0) &&
       (strcmp("/dev/ttyS2", argv[1]) != 0) &&
       (strcmp("/dev/ttyS3", argv[1]) != 0) &&
       (strcmp("/dev/ttyS4", argv[1]) != 0) &&
       (strcmp("/dev/ttyS5", argv[1]) != 0)
     ) ||
      ((
           (strcmp("emitter", argv[2]) != 0) &&
           (strcmp("Emitter", argv[2]) != 0)) &&
       ((strcmp("receiver", argv[2]) != 0) &&
        (strcmp("Receiver", argv[2]) != 0))))
  {
    printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\nreceiver/emitter\n");
    exit(1);
  }

  /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
  */

  fd = open(argv[1], O_RDWR | O_NOCTTY);
  if (fd < 0)
  {
    perror(argv[1]);
    exit(-1);
  }

  if (tcgetattr(fd, &oldtio) == -1)
  { /* save current port settings */
    perror("tcgetattr");
    exit(-1);
  }

  if (!strcmp(argv[2], "emitter") || !strcmp(argv[2], "Emitter"))
    emitter = true;
  else
    emitter = false;

  if(argc == 4){
    char buf[255] = "./";
    strcat(buf, argv[3]);
    strcpy(FICHEIRO , buf);
  }

  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;

  /* set input mode (non-canonical, no echo,...) */
  newtio.c_lflag = 0;

  newtio.c_cc[VTIME] = 1; /* inter-character timer unused */
  newtio.c_cc[VMIN] = 0;  /* blocking read until 0 chars received */

  if (tcsetattr(fd, TCSANOW, &newtio) == -1)
  {
    perror("tcsetattr");
    exit(-1);
  }

  srand(time(NULL));

  //****************************************
  tcflush(fd, TCIOFLUSH);

  if (connect(fd)) {printf("Erro, connecting retornou 1\n"); goto END;}

  sleep(1);
  tcflush(fd, TCIOFLUSH);

  struct timespec start, end;
  clock_gettime(CLOCK_MONOTONIC, &start);

  if (transfer_file(fd)) {printf("Erro, transfer_file retornou 1\n"); goto END;}

  clock_gettime(CLOCK_MONOTONIC, &end);

  if(ending(fd)){printf("Erro, disconnecting retornou 1\n"); goto END;}

  sleep(1);
  tcflush(fd, TCIOFLUSH);

  if(!emitter){
      if(transferedSize != fileSize){
        printf("There were transfer error:\n\tFile size: %d\n\tTransfered size: %d\n", fileSize, transferedSize);
      }
      else{
        printf("Success\n");
      }
    }

  double time_taken = (end.tv_sec - start.tv_sec); // seconds
  time_taken += (end.tv_nsec - start.tv_nsec) * 1e-9; // adds nanoseconds
  //printf("Took: %.3f seconds\nSent %d bytes at %.2f bps\n", time_taken, bytesSent, bytesSent/time_taken);

  //****************************************

END:

  tcflush(fd, TCIOFLUSH);

  sleep(1);

  if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
  {
    perror("tcsetattr");
    exit(-1);
  }

  close(fd);
  return 0;
}
