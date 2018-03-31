package biomight.system.ai.nlp.beans;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

import org.apache.openejb.math.util.MathUtils;

public class ContextBeach {

	int wordPhraseCount;
	private HashMap wordMap = new HashMap<String, String>();
	private ArrayList responseList = new ArrayList< String>();
	
	/******************************************************************************
	 * CONSTRUCTORS
	 * 
	 * 
	 * 
	 ******************************************************************************/
	public ContextBeach()
	{
		System.out.println("Running Default Constructor");
	}
	

	public ContextBeach(String[] tokens, String[] tags)
	{
		System.out.println("Executing ContextHello(TAGS) constructor...");
		loadWordMap();
		checkSyntax(tokens, tags);
		loadresponseList();
	}
	
	public ContextBeach(String tag)
	{
		System.out.println("Executing ContextHello(TAG) constructor...");
		//checkHello(tags);
	}

	
	public ContextBeach(int tag)
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
		
		System.out.println("Checking for Hello Context with TAGS...");
		
		// We will apply the tokens to the max of wordmap starting
		// at largest phrases and matching to the smallest.  
		String tokenPhrase = "";
		for (int j=0; j<tokens.length; j++)
		{		
			
			// Collect up the tokens based on the token count
			// in the phrase or utterance
			
			for (int i=j; i<(j+wordPhraseCount); i++)
			{
				if (i == (j+wordPhraseCount-1))
				{
					tokenPhrase+= tokens[i];
					System.out.println("Last Word: " + j +  "  " + i + "   " + tokenPhrase);
				}
				else
				{
					tokenPhrase+= tokens[i];
					tokenPhrase+= " ";
					System.out.println("Need Spaces: " + j +  "  " + i + "   " + tokenPhrase);
				}
				
				System.out.println("Phrase with Count: " + j +  "  " + i + "   " + tokenPhrase);
				
				if ( wordMap.containsKey(tokenPhrase) ) {
					prob = 100.00;
					break;
				}
			}
		
		}
		System.out.println("Prob of saying hello: " + prob);			
		return prob;
		
	}
	
	
	public void loadWordMap()
	{
		wordMap.put("Hi", "Greet");	
		wordMap.put("Hello", "Greet");
		wordMap.put("Good Morning", "Greet");
		wordMap.put("Good Day", "Greet");
		wordMap.put("Hey", "Greet");
		wordMap.put("What's up?", "Greet");
		wordPhraseCount = 2;
	}
	
	
	public void loadresponseList()
	{
		responseList.add("Hey man, what's up?");	
		responseList.add("Hello back to you");
		responseList.add("Good Day friend");
		responseList.add("Hey Jim");
		responseList.add("Hey");
		responseList.add("Fk off, I'm busy");
		responseList.add("What up punk?");
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
	
	public ArrayList<String> getresponseList() {
		return responseList;
	}


	public void setresponseList(ArrayList<String> responseList) {
		this.responseList = responseList;
	}
	
	
	
	
	
}
