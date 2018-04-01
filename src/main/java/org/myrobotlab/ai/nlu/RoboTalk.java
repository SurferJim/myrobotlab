package org.myrobotlab.ai.nlu;

import java.io.FileInputStream;
import java.io.InputStream;
import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;

import opennlp.tools.chunker.ChunkerME;
import opennlp.tools.chunker.ChunkerModel;
import opennlp.tools.lemmatizer.LemmatizerME;
import opennlp.tools.lemmatizer.LemmatizerModel;
import opennlp.tools.namefind.NameFinderME;
import opennlp.tools.namefind.TokenNameFinderModel;
import opennlp.tools.parser.ParserModel;
import opennlp.tools.postag.POSModel;
import opennlp.tools.postag.POSTaggerME;
import opennlp.tools.sentdetect.SentenceDetectorME;
import opennlp.tools.sentdetect.SentenceModel;
import opennlp.tools.tokenize.Tokenizer;
import opennlp.tools.tokenize.TokenizerME;
import opennlp.tools.tokenize.TokenizerModel;
import opennlp.tools.util.Sequence;
import opennlp.tools.util.Span;
import oracle.sql.DATE;



public class RoboTalk {
	

	ArrayList responseList = new ArrayList();
	
	// Context Frames.  These java object represent their real world
	// counterparts.  Each holds properties and methods that reflect 
	// the real world use
	private String[] contextModels = {
			"org.myrobotlab.nlu.ContextHello",
			"org.myrobotlab.nlu.ContextTime",
			"org.myrobotlab.nlu.ContextTrain",
			"org.myrobotlab.nlu.ContextBeach",
			"org.myrobotlab.nlu.ContextLunch"
	};

	
	
	//***********************************************************************
	//
	// PROCESS REQUEST
	//
	//***********************************************************************
	public ArrayList processRequest(String myRequest)
	{	
		System.out.println("Request is: " + myRequest + "\n");
		
		InputStream modelIn = null;
		String modelsPath = "C://OpenNlp//models//";
		String modelLocation = modelsPath + "en-pos-maxent.bin";
		
		try {
			
			//**********************************************************
			// Perform Sentence Tokenization
			//**********************************************************
			modelLocation = modelsPath + "en-sent.bin";
			modelIn = new FileInputStream(modelLocation);
			SentenceModel sentModel = new SentenceModel(modelIn);
			SentenceDetectorME sentenceDetector = new SentenceDetectorME(sentModel);
			String sentences[] = sentenceDetector.sentDetect(myRequest);
			
			// Output the Sentences
			System.out.println("Sentences...");
			for (int i=0; i<sentences.length; i++)
			{
				System.out.println(i + ".  " + sentences[i]);
			}
			
		
			//**********************************************************
			// Perform Word Tokenization on the sentences
			//**********************************************************
			for (int j=0; j<sentences.length; j++)
			{
				modelLocation = modelsPath + "en-token.bin";
				modelIn = new FileInputStream(modelLocation);
				TokenizerModel tokenModel = new TokenizerModel(modelIn);
				Tokenizer tokenizer = new TokenizerME(tokenModel);
				String tokens[] = tokenizer.tokenize(sentences[j]);
				
				// Output the words in a Sentences
				System.out.println("\nTokens for sentence...." + j);
				for (int i=0; i<tokens.length; i++)
				{
					System.out.println(i + ".  " + tokens[i]);
				}
				
				//**********************************************************
				// Perform NameEntity Recognition
				//**********************************************************
				modelLocation = modelsPath + "en-ner-person.bin";
				InputStream modelNER = (InputStream) new FileInputStream(modelLocation);
				TokenNameFinderModel  finderModel = new TokenNameFinderModel(modelNER);
				NameFinderME nameFinder = new NameFinderME(finderModel);
				
				Span names[] = nameFinder.find(tokens);
				
				// Output the NER
				System.out.println("\nNER...");
				for (int i=0; i<names.length; i++)
				{
					System.out.println(i + ".  " + names[i]);
				}
		
		
				//**********************************************************
				// Perform POS Tagging
				//**********************************************************
				modelLocation = modelsPath + "en-pos-maxent.bin";
				modelIn = new FileInputStream(modelLocation);
				POSModel model = new POSModel(modelIn);
				
				POSTaggerME tagger = new POSTaggerME(model);
				String tags[] = tagger.tag(tokens);
				double probs[] = tagger.probs();
				
				// Output the POS
				System.out.println("\nPOS...");
				for (int i=0; i<tags.length; i++)
				{
					System.out.println(i + ".  " + tags[i]);
				}
		
				// Output the POS probabilities
				System.out.println("\nProbs...");
				for (int i=0; i<probs.length; i++)
				{
					System.out.println(i + ".  " + probs[i]);
				}
		
				//**********************************************************
				// Perform Lemmatization
				//**********************************************************
				/*
				LemmatizerModel lemmaModel = null;
				modelIn = new FileInputStream("en-lemmatizer.bin");
			    lemmaModel = new LemmatizerModel(modelIn);
			    LemmatizerME lemmatizer = new LemmatizerME(lemmaModel);
			    
			    String[] lemmas = lemmatizer.lemmatize(tokens, tags);
				// Output the LEMMA probabilities
				System.out.println("\nLemmas...");
				for (int i=0; i<lemmas.length; i++)
				{
					System.out.println(i + ".  " + lemmas[i]);
				}
				*/
				
				//**********************************************************
				// Text Chunking
				//**********************************************************
				ChunkerModel chunkModel = null;

				modelLocation = modelsPath + "en-chunker.bin";
				modelIn = new FileInputStream(modelLocation);
				chunkModel = new ChunkerModel(modelIn);
				
				ChunkerME chunker = new ChunkerME(chunkModel);
				
				String chunks[] = chunker.chunk(tokens, tags);
				
				// Output the Word Chunks
				System.out.println("\nChunks...");
				for (int i=0; i<chunks.length; i++)
				{
					System.out.println(i + ".  " + chunks[i]);
				}
		
			
				// Try and determine what the user is talking about
				 responseList = determineContext(tokens, tags);
				 
			}
					
		}
		catch (Exception e)
		{
			System.out.println("Talk Exception: " + e);
		}

		//myResponse = getTime();
		return(responseList);
		

	}
	

	
	//*****************************************************
	// Determine Context 
	// 
	// This method will apply the various contexts against the 
	// sentence to...
	//*****************************************************
	public ArrayList<String> determineContext(String[] tokens, String[] tags) {
		
		try 
		{
				
			// Fire off the various context frames to see if anything sticks
			for (int i=0; i<1; i++)
			{
				
				Class clazz  = null;
				Object runObject = null;
				
			
				try {
				 System.out.println("Loading ContextModel: " + contextModels[i]);
				 Constructor cons = Class.forName(contextModels[i]).getDeclaredConstructor( new Class[]{String[].class, String[].class});
				 
				// System.out.println("Constructor method is identified! " + cons.getName() + "  " + cons.getParameterCount() );
				// System.out.println("Constructor Params matched:  " + cons.getParameterTypes());
				 
				 if (cons == null) 
				 {
					 System.out.println("Constructor could not be created as obj is null"); 
				 }
				 else 
				 {
					 
					 try {
						 Object newInstance = cons.newInstance((Object)tokens, (Object)tags);
						 System.out.println("NewInstance Context Object created via Constructor");
						 
						 Class noParams[] = {};
						 Method method = newInstance.getClass().getDeclaredMethod("getMyResponse",  noParams);
						 //System.out.println("Response method is identified! " + method.getName() + "  " + method.getParameterCount() );
						 //System.out.println("Response Params matched:  " + method.getParameterTypes());
						 
						 //System.out.println("Calling Invoke on getMyResponse()");
						 String myResponse =  (String) method.invoke(newInstance, null);
						 
						 if (!myResponse.isEmpty()) {
							 System.out.println("Adding Response: " + myResponse);
							 if (myResponse != null)
								 responseList.add(myResponse);
						 }
						 
					 }
					 catch (Exception e)
					{
						System.out.println("Could not create Java Method reference " + contextModels[i] + " " + e);
					}
					 
				 }
				
				 
				}
				catch (Exception e)
				{
					System.out.println("Could not create Java Class " + contextModels[i]);
				}

			}
				
		} 
		catch (Exception e) 
		{
			System.out.println("ERROR in determining Context: " + e);	
		}
		
		
		return responseList;
	}
	
	

	
	// TIME CONTEXT
	public String getTime() {
		
		String timeStr = "";

		
		try {
			return ("The time is: " + DATE.getCurrentDate() );
		}
		catch (Exception e)
		{
			System.out.println("Talk Exception: " + e);
		}
		
		return timeStr;
	}
	
	

}




