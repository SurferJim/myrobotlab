package org.myrobotlab.nlu;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

import org.apache.openejb.math.util.MathUtils;

public class ContextHello {

	int wordPhraseCount;
	private ArrayList<String[]> wordList = new ArrayList<String[]>();
	private ArrayList<String> responseList = new ArrayList< String>();
	
	/******************************************************************************
	 * CONSTRUCTORS
	 * 
	 * 
	 * 
	 ******************************************************************************/
	public ContextHello()
	{
		System.out.println("Running Default Constructor");
	}
	

	public ContextHello(String[] tokens, String[] tags)
	{
		System.out.println("Executing ContextHello(TAGS) constructor...");
		loadVocab();
		applyContext(tokens, tags);
		loadresponseList();
	}
	
	public ContextHello(String tag)
	{
		System.out.println("Executing ContextHello(TAG) constructor...");
		//checkHello(tags);
	}

	
	public ContextHello(int tag)
	{
		System.out.println("Executing ContextHello(TAG0) constructor...");
		//checkHello(tags);
	}
	
	
	/*************************************************************
	 * APPLY CONTEXT
	 * 
	 * @return
	 *************************************************************/

	public double applyContext(String[] tokens, String[] tags)
	{
		double prob = 0.0;
		
		System.out.println("Apply HelloContext.."); 
		
		// Run through the tokens that we received as input
		int tokenLength = tokens.length;
		
		// This will assemble phrases from the user's input
		// The phrase starts at one word, and then appends words until
		// entire phrase is consumed
		String[] tokenPhrase = new String[tokenLength];

		// Run once through every token in the user's input sentence
		for (int j=0; j<tokenLength; j++)
		{		
			int count = 0;
			for (int i=j; i<tokens.length; i++)
			{
				tokenPhrase[count++] = tokens[i];
				
				listTokens("Apply Context to UserPhrase: ", tokenPhrase);
			
				prob = doMatch(tokenPhrase);
			}	
		
		}
		System.out.println("Prob of saying hello: " + prob);			
		return prob;
		
	}
	
	
	/********************************************************************
	 * DO MATCH
	 * 
	 * @param tokenPhrase
	 * @return
	 ********************************************************************/
	
	public double doMatch(String[] tokenPhrase)
	{
		double prob = 0.00;
		
		// Run through the associated words and phrases for this context
		// to see what matches up in the user's input request.  
		for (int i=0; i<wordList.size(); i++)
		{
			String[] cannedPhrase = (String[]) wordList.get(i);
			listTokens("DoMatch...Canned Phrase: ", cannedPhrase);
			int tokPhraseLen = tokenPhrase.length;

			for (int j=0; j<cannedPhrase.length; j++)
			{
				if (j<tokPhraseLen)
				{
					if (tokenPhrase[j] != null && cannedPhrase[j] != null)
					{	
						if (cannedPhrase[j].equalsIgnoreCase(tokenPhrase[j]))  
						{
							prob = 100.00;
							System.out.println("Matched User tokens: " + tokenPhrase[j] + " canned: "  + cannedPhrase[j]);	
						}
						else
							System.out.println("NoMatch User tokens: " + tokenPhrase[j] + " canned "  + cannedPhrase[j]);	
					}	
				}
			}
		}
		
		System.out.println("Prob of saying hello: " + prob);			
		return prob;
	}




	// Just grab a random response, later we will 
	public String getMyResponse()
	{
		// nextInt is normally exclusive of the top value,
		// so add 1 to make it inclusive
		int randomNum = ThreadLocalRandom.current().nextInt(1, 6);
		System.out.println("Random # is : " + randomNum);
		String myResponse = (String) responseList.get(randomNum);
		System.out.println("Random # is : " + myResponse);
		return (myResponse);
		
	}

	
	//*********************************************************************
	// Output the token list
	//*********************************************************************
	public void listTokens(String msgHeader, String[] tokens)
	{
		System.out.println("\n");
		System.out.println(msgHeader);
		
		for (int i=0; i<tokens.length; i++)
		{
			if (tokens[i] != null)
			{
				System.out.println("index: " + i + "   " + tokens[i]);
			}
		}	
		System.out.println("\n");
	}
	
	
	
	public ArrayList<String> getresponseList() {
		return responseList;
	}


	public void setresponseList(ArrayList<String> responseList) {
		this.responseList = responseList;
	}
	
	
	/***************************************************************************
	 * LOAD VOCAB
	 * 
	 * A generic bunch of canned input phrases
	 * 
	 **************************************************************************/
	public void loadVocab()
	{
 		wordList.add( new String[] {"Hi"});	
 		wordList.add( new String[] {"Hello"});	
 		wordList.add( new String[] {"Good", "Morning"});	
 		wordList.add( new String[] {"Good", "Day"});
 		wordList.add( new String[] {"Good", "Afternoon"});
 		wordList.add( new String[] {"Good", "Evening"});
 		wordList.add( new String[] {"Hey"});
 		wordList.add( new String[] {"What's", "Up", "?"});
	}
	
	
	/***************************************************************************
	 * LOAD RESPONSES
	 * 
	 * A generic bunch of canned responses.
	 * 
	 **************************************************************************/
	public void loadresponseList()
	{
		responseList.add("Hey man, what's up?");	
		responseList.add("Hello back to you");
		responseList.add("Good Day friend");
		responseList.add("Hey Robo Maker");
		responseList.add("Hey");
		responseList.add("What up punk?");
		responseList.add("Hello Creator");
	}

	
}
