package biomight.system.ai.nlp;

import java.io.FileInputStream;
import java.io.InputStream;
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

public class RoboTalk {

	
	
	public String processRequest(String myRequest)
	{
		String myResponse = "Hello";
		
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
				LemmatizerModel lemmaModel = null;
				modelIn = new FileInputStream("en-lemmatizer.bin");
			    lemmaModel = new LemmatizerModel(modelIn);
			    LemmatizerME lemmatizer = new LemmatizerME(lemmaModel);
			    
			    String[] lemmas = lemmatizer.lemmatize(tokens, tags);
				// Output the LEMMA probabilities
				System.out.println("\nProbs...");
				for (int i=0; i<probs.length; i++)
				{
					System.out.println(i + ".  " + probs[i]);
				}
		
				
				//**********************************************************
				// Text Chunking
				//**********************************************************
				ChunkerModel chunkModel = null;

				modelLocation = modelsPath + "en-chunker.bin";
				System.out.println("Chunker Model has been loaded: " + modelLocation);
				modelIn = new FileInputStream(modelLocation);
				System.out.println("Chunker Model opened Stream...");
				chunkModel = new ChunkerModel(modelIn);
				System.out.println("Chunker Model intiaized...");
				
				
				ChunkerME chunker = new ChunkerME(chunkModel);
				System.out.println("Chunker ModelME has been loaded...");
				
				String chunks[] = chunker.chunk(tokens, tags);
				
				
				// Output the Word Chunks
				System.out.println("\nChunks...");
				for (int i=0; i<chunks.length; i++)
				{
					System.out.println(i + ".  " + chunks[i]);
				}
		
			
				
			}
					
		}
		catch (Exception e)
		{
			System.out.println("Talk Exception: " + e);
		}

		return(myResponse);
	}
	
	
	
	public String getResponse(String[] request)
	{
		String myResponse = "";	
		return (myResponse);
	}
	
	
	public void matchScenario() {  
	
			HashMap contextMap = new HashMap();
			String[] tokList =  {"Hi", "Hello", "What's up?"};
			contextMap.put("greet", tokList);	
	}
	
	
	

}




