/*******************************************************************************
 Ficheiro de implementação do Tipo de Dados Abstracto SEQ_LIST (seqlist.c).
 A estrutura de dados de suporte da sequência é uma estrutura, constituída pelos
 campos de tipo inteiro Size para indicar o número de elementos armazenados na
 sequência e os campos de tipo ponteiro para nós de lista biligada Head e Tail,
 para representar, respectivamente, a cabeça e a cauda da lista biligada onde
 são armazenados os números inteiros.

 Autor : Gonçalo Vítor			                       NMEC :  85119         
*******************************************************************************/

#include <stdio.h> 
#include <stdlib.h>

#include "seqlist.h"  /* Ficheiro de interface do TDA - ADT Interface file */

/************ Definição da Estrutura de Dados Interna da Sequência ************/

typedef struct binode *PtBiNode;
struct binode /* definição do nó da lista biligada */
{
	int Elem; /* o elemento da lista */
	PtBiNode PtPrev, PtNext; /* ponteiros para o nós anterior e seguinte */
};

struct seqlist
{
  int Size; /* número de elementos - sequence's size */
  PtBiNode Head; /* ponteiro para o início da lista (cabeça da lista) - list head */
  PtBiNode Tail; /* ponteiro para o fim da lista (cauda da lista) - list tail */
};

/*********************** Controlo Centralizado de Error ************************/

static unsigned int Error = OK;  /* inicialização do erro */

static char *ErrorMessages[] = {
                                 "sem erro - Without Error",
                                 "sequencia(s) inexistente(s) - Sequence(s) do not exist",
                                 "memoria esgotada - Out of memory",
                                 "indice errado - Wrong index",
                                 "elemento inexistente - Element does not exist",
                                 "sequencia vazia - Empty sequence",
                                 "sequencia cheia - Full sequence",
                                 "dimensao da sequencia errada - Wrong size",
                                 "ficheiro inexistente - File does not exist",
                                 "ponteiro nulo - Null pointer"
                               };

static char *AbnormalErrorMessage = "erro desconhecido - Unknown error";

/************** Número de mensagens de erro previstas no módulo ***************/

#define N (sizeof (ErrorMessages) / sizeof (char *))

/******************** Protótipos dos Subprogramas Internos ********************/

PtBiNode BiNodeCreate (int);
void BiNodeDestroy (PtBiNode *);
void DoubleListDestroy (PtBiNode *);

/*************************** Definição das Funções ****************************/

void SeqListClearError (void)
{ Error = OK; }

int SeqListError (void)
{ return Error; }

char *SeqListErrorMessage (void)
{
  if (Error < N) return ErrorMessages[Error];
  else return AbnormalErrorMessage;  /* sem mensagem de erro - no error message */
}

PtSeqList SeqListCreate ()
{
  
	PtSeqList seqlist; 

	/* cria a seq - allocating the supporting record */
	if((seqlist = malloc(sizeof(struct seqlist))) == NULL)
	{ Error = NO_MEM; return NULL; }

	seqlist->Size = 0;
	seqlist->Head = NULL;
	seqlist->Tail = NULL;	

	Error = OK;
	return seqlist;  

}

void SeqListDestroy (PtSeqList *pseq)
{

	PtSeqList tmp = *pseq;
	
	/* Verificar se a seq existe */
	if(tmp == NULL) { Error = NO_SEQ; return; }

	DoubleListDestroy(&tmp->Head);
	free(tmp);

	Error = OK;
	*pseq = NULL;

}

PtSeqList SeqListCopy (PtSeqList pseq)
{
	PtSeqList Copy;

	/* verifica se a sequência existe - verifies if sequence exists */
	if (pseq == NULL) { Error = NO_SEQ; return NULL; }

	/* criação da sequência copia nulo - creating an empty sequence */
	if ((Copy = SeqListCreate ()) == NULL) return NULL;

	/* fazer a copia da sequência - copying the components */
	for (PtBiNode Node = pseq->Head; Node != NULL; Node = Node->PtNext)
	{
		SeqListInsert (Copy, Node->Elem);
		if (Error == NO_MEM) break;
	}

	if (Error == NO_MEM) { SeqListDestroy (&Copy); return NULL; }

	Copy->Size = pseq->Size;
	Error = OK;
	return Copy;  /* devolve a sequência copia - returning the new sequence */
}

PtSeqList SeqListFileCreate (char *pfname)
{
	PtSeqList Seq; FILE *PtF; unsigned int Size, I; int Elem;

	/* abertura com validacao do ficheiro para leitura - opening the text file for reading */
	if ( (PtF = fopen (pfname, "r")) == NULL) { Error = NO_FILE; return NULL; }

	/* leitura da dimensão da sequência e do número de elementos */
	/* reading the sequence's dimension and the number of elements */
	fscanf (PtF, "%u", &Size);
	if (Size < 1) { Error = BAD_SIZE; fclose (PtF); return NULL; }

	/* criação da sequência vazia - creating an empty sequence */
	if ((Seq = SeqListCreate ()) == NULL) { fclose (PtF); return NULL; }

	Error = OK;
	/* leitura da sequência do ficheiro - reading the sequence's components from the text file */
	for (I = 0; I < Size; I++)
	{
		fscanf (PtF, "%d", &Elem);
		SeqListInsert (Seq, Elem);
		if (Error == NO_MEM) break;
	}

	if (Error == NO_MEM) { SeqListDestroy (&Seq); return NULL; }

	fclose (PtF);  /* fecho do ficheiro - closing the text file */
	return Seq;  /* devolve a sequência criada - returning the new sequence */
}

void SeqListStoreFile (PtSeqList pseq, char *pfname)
{
	FILE *PtF;

	/* verifica se a sequência existe - verifies if sequence exists */
	if (pseq == NULL) { Error = NO_SEQ; return ; }

	/* verifica se a sequência tem elementos - verifies if sequence has elements */
	if (pseq->Size == 0) { Error = SEQ_EMPTY; return ; }

	/* abertura com validacao do ficheiro para escrita - opening the text file for writing */
	if ((PtF = fopen (pfname, "w")) == NULL) { Error = NO_FILE; return ; }

	/* escrita do número de elementos armazenados na sequência */
	/* writing the number of elements */
	fprintf (PtF, "%u\n", pseq->Size);

	/* escrita da sequência - writing the sequence's components in the text file */
	for (PtBiNode Node = pseq->Head; Node != NULL; Node = Node->PtNext)
		fprintf (PtF, "%d\n", Node->Elem);

	Error = OK;
	fclose (PtF);  /* fecho do ficheiro - closing the text file */
}

int SeqListGetSize (PtSeqList pseq)
{

	/* Verificar se a seq existe */	
	if(pseq == NULL) { Error = NO_SEQ; return 0; }
	
	Error = OK;
	return pseq->Size;

}

int SeqListGetElement (PtSeqList pseq, int pindex)
{

	/* Verificar se a seq existe */	
	if(pseq == NULL) { Error = NO_SEQ; return 0; }

	PtBiNode node;
	/* Verifica se o index é valido */
	if(pindex >= 0 && pindex < pseq->Size) {
		node = pseq->Head;
		for(int i = 0; i < pindex; i++)
			node = node->PtNext;
	} else if(pindex <= 0 && pindex > -(pseq->Size)) {
		node = pseq->Tail;
		for(int i = pseq->Size-1; i > pindex; i++)
			node = node->PtPrev;
	} else { 
		Error = BAD_INDEX; return 0; 
	}

	Error = OK;
	return node->Elem;

}

void SeqListSetElement (PtSeqList pseq, int pindex, int pvalue)
{

	/* Verificar se a seq existe */	
	if(pseq == NULL) { Error = NO_SEQ; return; }

	PtBiNode node;
	/* Verifica se o index é valido */
	if(pindex >= 0 && pindex < pseq->Size) {
		node = pseq->Head;
		for(int i = 0; i < pindex; i++)
			node = node->PtNext;
	} else if(pindex < 0 && pindex >= -(pseq->Size)) {
		node = pseq->Tail;
		for(int i = 1; i < -pindex; i++)
			node = node->PtPrev;
	} else { 
		Error = BAD_INDEX; return; 
	}

	Error = OK;
	node->Elem = pvalue;

}

int SeqListEquals (PtSeqList pseq1, PtSeqList pseq2)
{

	/* Verificar se as seqs existem */	
	if(pseq1 == NULL || pseq2 == NULL) { Error = NO_SEQ; return 0; }

	if(pseq1->Size != pseq2->Size) return 0;

	PtBiNode seq1node = pseq1->Head;
	PtBiNode seq2node = pseq2->Head;
	for(int i = 0; i < pseq1->Size; i++) {
		if(seq1node->Elem != seq2node->Elem) return 0;
		seq1node = seq1node->PtNext;
		seq2node = seq2node->PtNext;
	}

	Error = OK;
	return 1;

}

void SeqListInsert (PtSeqList pseq, int pvalue)
{

	/* Verificar se a sequencia existe */
	if(pseq == NULL) { Error = NO_SEQ; return; }

	/* Verificar se há espaco para alocar um node */
	PtBiNode tmp;
	if((tmp = BiNodeCreate(pvalue)) == NULL) { Error = NO_MEM; return; }

	if(pseq->Size == 0) pseq->Head = tmp;
	else { pseq->Tail->PtNext = tmp; tmp->PtPrev = pseq->Tail; }

	pseq->Tail = tmp;	
	pseq->Size++;
	Error = OK;

}

void SeqListDelete (PtSeqList pseq, int pvalue)
{

	/* Verificar se a sequencia existe */
	if(pseq == NULL) { Error = NO_SEQ; return; }

	/* Verificar se a lista tem nós */
	if(pseq->Size == 0) { Error = SEQ_EMPTY; return; }
	
	int sizeBefore = pseq->Size;

	for(PtBiNode node = pseq->Head; node != NULL;) {
		if(node->Elem == pvalue) {
			PtBiNode tmp = node->PtNext;
			if(node != pseq->Head) node->PtPrev->PtNext = node->PtNext;
			else pseq->Head = node->PtNext;
			if(node != pseq->Tail) node->PtNext->PtPrev = node->PtPrev;
			else pseq->Tail = node->PtPrev;
			BiNodeDestroy(&node);
			pseq->Size--;
			node = tmp; // Guarda para continuar a iterar na lista para apagar outros eventuais pvalues
		} else {
			node = node->PtNext;
		}
	}

	if(sizeBefore == pseq->Size) Error = NO_NUMBER;
	else Error = OK;

}

void SeqListRemoveLast (PtSeqList pseq) 
{

	/* Verificar se a sequencia existe */
	if(pseq == NULL) { Error = NO_SEQ; return; }

	/* Verificar se a lista tem nós */
	if(pseq->Size == 0) { Error = SEQ_EMPTY; return; }

	if(pseq->Size == 1) { 
		pseq->Tail = NULL;
		pseq->Head = NULL;
	} else {
		PtBiNode tmp = pseq->Tail;
		pseq->Tail = pseq->Tail->PtPrev;
		BiNodeDestroy(&tmp);
	}
	pseq->Size--;

	Error = OK;

}

int SeqListIsOrdered (PtSeqList pseq)
{

	/* Verificar se a sequencia existe */
	if(pseq == NULL) { Error = NO_SEQ; return 0; }

	/* Verificar se a lista tem nós */
	if(pseq->Size == 0) { Error = SEQ_EMPTY; return 0; }

	for(PtBiNode tmp = pseq->Head; tmp->PtNext != NULL; tmp = tmp->PtNext)
		if(tmp->Elem > tmp->PtNext->Elem) return 0;

	Error = OK;
	return 1;

}

int SeqListIsEvenOdd (PtSeqList pseq)
{
 	
	/* Verificar se a sequencia existe */
	if(pseq == NULL) { Error = NO_SEQ; return 0; }

	/* Verificar se a lista tem nós */
	if(pseq->Size == 0) { Error = SEQ_EMPTY; return 0; }
	
	int lastIsOdd = 0;
	for(PtBiNode tmp = pseq->Head; tmp != NULL; tmp = tmp->PtNext) {
		int currentIsOdd = (tmp->Elem % 2) == 0;
		if(tmp != pseq->Head && currentIsOdd == lastIsOdd) return 0;
		lastIsOdd = currentIsOdd;
	}

	Error = OK;
	return 1;

}

int SeqListNumberOfMultiples (PtSeqList pseq, int pvalue)
{

	/* Verificar se a sequencia existe */
	if(pseq == NULL) { Error = NO_SEQ; return 0; }

	/* Verificar se a lista tem nós */
	if(pseq->Size == 0) { Error = SEQ_EMPTY; return 0; }

	int count = 0;	
	for(PtBiNode tmp = pseq->Head; tmp != NULL; tmp = tmp->PtNext)
		if(tmp->Elem % pvalue == 0) count++;

	Error = OK;
	return count;


}

void SeqListSmallerBigger (PtSeqList pseq, int * psmall, int * pbig)
{

	/* Verificar se a sequencia existe */
	if(pseq == NULL) { Error = NO_SEQ; return; }

	/* Verificar se a lista tem nós */
	if(pseq->Size == 0) { Error = SEQ_EMPTY; return; }

	*psmall = pseq->Head->Elem, *pbig = pseq->Head->Elem;
	for(PtBiNode tmp = pseq->Head->PtNext; tmp != NULL; tmp = tmp->PtNext) {
		if(tmp->Elem < *psmall) *psmall = tmp->Elem;
		if(tmp->Elem > *pbig) *pbig = tmp->Elem;
	}

	Error = OK;

}

int SeqListIsPalindromic (PtSeqList pseq)
{

	/* Verificar se a sequencia existe */
	if(pseq == NULL) { Error = NO_SEQ; return 0; }

	/* Verificar se a lista tem nós */
	if(pseq->Size == 0) { Error = SEQ_EMPTY; return 0; }

	int end = pseq->Size - 1;
	PtBiNode tmpHead = pseq->Head;
	PtBiNode tmpTail = pseq->Tail;
	for(int i = 0; i < end; i++) {
		if(tmpHead->Elem != tmpTail->Elem) return 0;
		tmpHead = tmpHead->PtNext;
		tmpTail = tmpTail->PtPrev;
		end--;
	}

	Error = OK;
	return 1;

}

double SeqListAverage (PtSeqList pseq)
{

	/* Verificar se a sequencia existe */
	if(pseq == NULL) { Error = NO_SEQ; return 0.0; }

	/* Verificar se a lista tem nós */
	if(pseq->Size == 0) { Error = SEQ_EMPTY; return 0.0; }

	double sum = 0.0;
	for(PtBiNode tmp = pseq->Head; tmp != NULL; tmp = tmp->PtNext) 
		sum += tmp->Elem;

	Error = OK;
	return sum/pseq->Size;	

}

/*******************************************************************************
 Função auxiliar para criar um nó da lista biligada. Valores de erro: OK ou NO_MEM.
 
 Auxiliary function to create a binode. Error codes: OK or NO_MEM.
*******************************************************************************/

PtBiNode BiNodeCreate (int pelem)	/* alocação do nó */
{
	PtBiNode Node;

	if ((Node = (PtBiNode) malloc (sizeof (struct binode))) == NULL)
	{ Error = NO_MEM; return NULL; }

	Node->Elem = pelem;	/* copiar a informação */
	Node->PtPrev = NULL;	/* apontar para detrás para NULL */
	Node->PtNext = NULL;	/* apontar para a frente para NULL */

	Error = OK;
	return Node;
}

/*******************************************************************************
 Função auxiliar para libertar um nó da lista biligada. Valores de erro: OK ou NULL_PTR.
 
 Auxiliary function to free a binode. Error codes: OK or NULL_PTR.
*******************************************************************************/
void BiNodeDestroy (PtBiNode *pnode)	/* libertação do nó */
{
	if (*pnode == NULL) { Error = NULL_PTR; return; }
	free (*pnode);	/* libertação do nó */
	*pnode = NULL;	/* colocar o ponteiro a nulo */
	Error = OK;
}

/*******************************************************************************
 Função auxiliar para destruir uma lista biligada. Valores de erro: OK ou NULL_PTR.
 
 Auxiliary function to destroy a double link list. Error codes: OK or NULL_PTR.
*******************************************************************************/
void DoubleListDestroy (PtBiNode *phead)
{
	PtBiNode TmpHead = *phead; PtBiNode Node;

	if (TmpHead == NULL) { Error = NULL_PTR; return; }
	while (TmpHead != NULL)
	{
		Node = TmpHead; TmpHead = TmpHead->PtNext;
		BiNodeDestroy (&Node);
	}
	Error = OK; 
}

