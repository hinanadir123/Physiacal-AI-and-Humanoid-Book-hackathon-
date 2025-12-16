import os
import requests
from bs4 import BeautifulSoup
from dotenv import load_dotenv
import tiktoken
import cohere
from qdrant_client import QdrantClient, models
import anthropic
import google.generativeai as genai
from flask import Flask, request, jsonify
from flask_cors import CORS
import logging
import random

# Load environment variables from .env file
load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
ANTHROPIC_API_KEY = os.getenv("ANTHROPIC_API_KEY")  # For Claude
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")  # For Gemini

# Initialize Qdrant client
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Initialize clients only when needed (lazy initialization)
def get_cohere_client():
    if not COHERE_API_KEY:
        raise ValueError("COHERE_API_KEY is not set in environment variables")
    return cohere.Client(api_key=COHERE_API_KEY)

def get_claude_client():
    if not ANTHROPIC_API_KEY:
        return None
    return anthropic.Anthropic(api_key=ANTHROPIC_API_KEY)

def get_gemini_model():
    if not GEMINI_API_KEY:
        return None
    genai.configure(api_key=GEMINI_API_KEY)
    return genai.GenerativeModel('gemini-pro')

# Initialize LLM clients
claude_client = get_claude_client() if ANTHROPIC_API_KEY else None
if not ANTHROPIC_API_KEY:
    print("Warning: ANTHROPIC_API_KEY not found in environment variables")

gemini_model = get_gemini_model() if GEMINI_API_KEY else None
if not GEMINI_API_KEY:
    print("Warning: GEMINI_API_KEY not found in environment variables")

# Create Flask app
app = Flask(__name__)
CORS(app, resources={
    r"/api/*": {"origins": "*"},
    r"/chat": {"origins": "*"},
    r"/collections": {"origins": "*"},
    r"/collections/*": {"origins": "*"},
    r"/health": {"origins": "*"},
    r"/index": {"origins": "*"},
    r"/ingest": {"origins": "*"}
})  # Enable CORS for all routes with specific configurations

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def get_all_urls(base_url: str) -> list[str]:
    """
    Fetches all URLs from the sitemap.xml of the given Docusaurus site.
    """
    sitemap_url = f"{base_url}/sitemap.xml"
    placeholder_domain = "https://your-docusaurus-site.example.com"
    urls = []
    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()  # Raise an exception for bad status codes
        soup = BeautifulSoup(response.content, "xml")
        locs = soup.find_all("loc")
        raw_urls = [loc.get_text() for loc in locs]
        # Replace placeholder domain with actual base_url from the main function
        urls = [url.replace(placeholder_domain, base_url) for url in raw_urls]
    except requests.exceptions.RequestException as e:
        print(f"Error fetching sitemap: {e}")
    return urls


def ingest_url_content(url: str) -> dict:
    """
    Ingests content from a single URL with proper metadata and section identifiers.
    Returns a dictionary with URL, content, and metadata.
    """
    content = extract_text_from_url(url)

    if not content:
        return {
            "url": url,
            "content": "",
            "section_id": "",
            "status": "failed",
            "word_count": 0,
            "char_count": 0
        }

    # Create section identifiers based on URL structure
    import re
    # Extract meaningful section name from URL
    path_parts = url.rstrip('/').split('/')
    section_id = path_parts[-1] if path_parts[-1] else path_parts[-2] if len(path_parts) > 1 else "home"

    # Sanitize section ID to make it a valid identifier
    section_id = re.sub(r'[^a-zA-Z0-9_-]', '_', section_id)

    return {
        "url": url,
        "content": content,
        "section_id": section_id,
        "status": "success",
        "word_count": len(content.split()),
        "char_count": len(content)
    }


def ingest_multiple_urls(urls: list[str]) -> list[dict]:
    """
    Ingests content from multiple URLs with deterministic ordering.
    Returns a list of content dictionaries with metadata.
    """
    results = []

    for url in urls:
        result = ingest_url_content(url)
        results.append(result)

    return results

def extract_text_from_url(url: str) -> str:
    """
    Extracts clean text from a given URL, excluding navigation, footer, and code blocks.
    """
    try:
        response = requests.get(url)
        response.raise_for_status()
        soup = BeautifulSoup(response.content, "html.parser")

        # Remove navigation elements
        for nav in soup.find_all(['nav', 'header', 'footer']):
            nav.decompose()

        # Remove sidebar navigation
        for sidebar in soup.find_all(class_=['menu', 'sidebar', 'navigation']):
            sidebar.decompose()

        # Remove code blocks and preformatted text
        for code_elem in soup.find_all(['pre', 'code', 'kbd', 'samp']):
            code_elem.decompose()

        # Remove script and style elements
        for script in soup.find_all(['script', 'style']):
            script.decompose()

        # Remove comments
        from bs4 import Comment
        for comment in soup.find_all(string=lambda text: isinstance(text, Comment)):
            comment.extract()

        # Docusaurus typically wraps main content in an <article> tag
        article = soup.find("article")
        if article:
            # Remove any remaining unwanted elements within the article
            for unwanted in article.find_all(['nav', 'header', 'footer', 'aside']):
                unwanted.decompose()
            return article.get_text(separator="\n", strip=True)

        # Look for main content containers with common Docusaurus class names
        main_content = soup.find("main") or soup.find(class_="main") or soup.find(id="main")
        if main_content:
            # Remove any remaining unwanted elements within the main content
            for unwanted in main_content.find_all(['nav', 'header', 'footer', 'aside']):
                unwanted.decompose()
            return main_content.get_text(separator="\n", strip=True)

        # Look for content containers
        content = soup.find(class_="container") or soup.find(class_="content") or soup.find(class_="doc-content")
        if content:
            for unwanted in content.find_all(['nav', 'header', 'footer', 'aside']):
                unwanted.decompose()
            return content.get_text(separator="\n", strip=True)

        # Fallback to body content
        for unwanted in soup.body.find_all(['nav', 'header', 'footer', 'aside']):
            unwanted.decompose()
        return soup.body.get_text(separator="\n", strip=True)
    except requests.exceptions.RequestException as e:
        print(f"Error fetching URL {url}: {e}")
        return ""
    except Exception as e:
        print(f"Error processing URL {url}: {e}")
        return ""

import tiktoken

def chunk_text(text: str, chunk_size: int = 700, overlap: int = 120, doc_start_id: int = 0) -> list[dict]:
    """Chunks text into smaller pieces with deterministic IDs."""
    if not text:
        return []

    encoding = tiktoken.get_encoding("cl100k_base")
    tokens = encoding.encode(text)

    chunks = []
    for i, pos in enumerate(range(0, len(tokens), chunk_size - overlap)):
        chunk_tokens = tokens[pos:pos + chunk_size]
        chunk_text = encoding.decode(chunk_tokens)
        # Create unique integer ID by combining document start ID with chunk position
        chunk_id = doc_start_id + i
        chunks.append({
            "id": chunk_id,
            "text": chunk_text
        })
    return chunks

def embed(chunks: list[dict], batch_size: int = 64) -> list[dict]:
    """Generates Cohere embeddings for text chunks in batches."""
    if not COHERE_API_KEY:
        raise ValueError("COHERE_API_KEY is not set.")

    cohere_client = get_cohere_client()
    embedded_chunks = []
    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i + batch_size]
        texts = [item["text"] for item in batch]
        try:
            # Note: The 'model' parameter may need to be adjusted based on Cohere's latest offerings.
            # 'embed-english-v3.0' is a common choice.
            response = cohere_client.embed(texts=texts, model="embed-english-v3.0", input_type="search_document")
            embeddings = response.embeddings

            for j, item in enumerate(batch):
                item["embedding"] = embeddings[j]
                embedded_chunks.append(item)

        except Exception as e:
            print(f"Cohere API error: {e}")
            # Simple retry logic can be added here if needed
            # For now, we'll just skip the batch on error
            continue

    return embedded_chunks

def create_collection(collection_name: str = "rag_embedding"):
    """Creates a Qdrant collection if it doesn't exist."""
    if not QDRANT_URL:
        raise ValueError("QDRANT_URL is not set.")

    try:
        # Check if collection already exists
        try:
            qdrant.get_collection(collection_name)
            print(f"Collection '{collection_name}' already exists, skipping creation.")
            return
        except:
            # Collection doesn't exist, so create it
            pass

        qdrant.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
        )
        print(f"Collection '{collection_name}' created successfully.")
    except Exception as e:
        print(f"Error creating Qdrant collection: {e}")
        raise

def save_chunk_to_qdrant(chunks: list[dict], collection_name: str = "rag_embedding"):
    """Upserts a batch of chunks with their embeddings to Qdrant."""
    if not QDRANT_URL:
        raise ValueError("QDRANT_URL is not set.")

    points = []
    for chunk in chunks:
        # Ensure the chunk has an embedding
        if "embedding" not in chunk:
            continue

        points.append(
            models.PointStruct(
                id=chunk["id"],
                vector=chunk["embedding"],
                payload={"text": chunk["text"]},
            )
        )

    if points:
        try:
            qdrant.upsert(
                collection_name=collection_name,
                points=points,
                wait=True,
            )
            print(f"Upserted {len(points)} points to '{collection_name}'.")
        except Exception as e:
            print(f"Error upserting to Qdrant: {e}")
            raise


def search_qdrant(query: str, collection_name: str = "rag_embedding", top_k: int = 5):
    """Searches Qdrant for relevant chunks based on the query."""
    if not COHERE_API_KEY:
        raise ValueError("COHERE_API_KEY is not set.")

    logger.info(f"Searching in collection: {collection_name} with query: {query[:50]}...")
    try:
        cohere_client = get_cohere_client()
        # Generate embedding for the query
        response = cohere_client.embed(texts=[query], model="embed-english-v3.0", input_type="search_query")
        query_embedding = response.embeddings[0]

        # Search in Qdrant using the correct method name and parameter (query_points)
        search_results = qdrant.query_points(
            collection_name=collection_name,
            query=query_embedding,
            limit=top_k
        ).points

        logger.info(f"Qdrant search returned {len(search_results)} results")

        # Extract relevant chunks
        relevant_chunks = []
        for result in search_results:
            chunk_text = result.payload.get("text", "")
            score = result.score
            logger.info(f"Found chunk with score: {score}, text preview: {chunk_text[:100]}...")
            relevant_chunks.append({
                "text": chunk_text,
                "score": score
            })

        logger.info(f"Returning {len(relevant_chunks)} relevant chunks")
        return relevant_chunks
    except Exception as e:
        logger.error(f"Error searching Qdrant: {e}")
        return []


def generate_response_claude(context: str, query: str):
    """Generate response using Claude with the provided context."""
    client = get_claude_client()
    if not client:
        return "Error: Claude API key not configured."

    try:
        prompt = f"""Based on the following context, please answer the question. If the context doesn't contain enough information to answer the question, please say so.

Context:
{context}

Question: {query}

Answer:"""

        message = client.messages.create(
            model="claude-3-sonnet-20240229",
            max_tokens=1000,
            temperature=0.7,
            system="You are a helpful assistant that answers questions based on the provided context. Only use information from the context to answer the question. If the context doesn't contain enough information, say so.",
            messages=[
                {
                    "role": "user",
                    "content": prompt
                }
            ]
        )

        return message.content[0].text
    except Exception as e:
        print(f"Error generating Claude response: {e}")
        return f"Error generating response: {str(e)}"


def generate_response_gemini(context: str, query: str):
    """Generate response using Gemini with the provided context."""
    model = get_gemini_model()
    if not model:
        return "Error: Gemini API key not configured."

    try:
        prompt = f"""Based on the following context, please answer the question. If the context doesn't contain enough information to answer the question, please say so.

Context:
{context}

Question: {query}

Answer:"""

        response = model.generate_content(prompt)
        return response.text
    except Exception as e:
        print(f"Error generating Gemini response: {e}")
        return f"Error generating response: {str(e)}"


def generate_follow_up_questions(query: str, context: str):
    """Generate follow-up questions based on the query and context to guide learning."""
    import re

    # Define common robotics/AI topics to help generate relevant follow-up questions
    topic_indicators = {
        'ros': ['ros', 'ros2', 'node', 'topic', 'service', 'launch', 'rclpy', 'urdf', 'package'],
        'ai': ['ai', 'machine learning', 'neural', 'algorithm', 'learning', 'model', 'training'],
        'simulation': ['gazebo', 'simulation', 'unity', 'isaac', 'sim'],
        'navigation': ['navigation', 'nav2', 'path', 'planning', 'move', 'motion'],
        'robotics': ['robot', 'robotic', 'actuator', 'sensor', 'control', 'manipulator']
    }

    query_lower = query.lower()
    context_lower = context.lower()

    # Determine which topics are relevant to generate appropriate follow-up questions
    relevant_topics = []
    for topic, keywords in topic_indicators.items():
        if any(keyword in query_lower or keyword in context_lower for keyword in keywords):
            relevant_topics.append(topic)

    # Generate follow-up questions based on detected topics
    follow_up_templates = []

    if 'ros' in relevant_topics:
        follow_up_templates.extend([
            "How do I implement this in a ROS 2 package?",
            "What are the best practices for creating ROS 2 nodes?",
            "How does this relate to ROS 2 concepts like topics and services?",
            "Can you show me a practical example of this in a ROS 2 system?"
        ])
    elif 'ai' in relevant_topics:
        follow_up_templates.extend([
            "How can I improve the AI model's performance?",
            "What are the ethical considerations for this AI application?",
            "How does this AI technique compare to other methods?",
            "What are the limitations of this AI approach?"
        ])
    elif 'simulation' in relevant_topics:
        follow_up_templates.extend([
            "How do I set up my simulation environment?",
            "What parameters should I tune for better simulation results?",
            "How can I transfer what I learn in simulation to real robots?",
            "What are the differences between various simulation platforms?"
        ])
    elif 'navigation' in relevant_topics:
        follow_up_templates.extend([
            "How do I configure the navigation stack for my robot?",
            "What sensors are needed for effective navigation?",
            "How can I make the navigation more robust in dynamic environments?",
            "What are common troubleshooting steps for navigation issues?"
        ])
    elif 'robotics' in relevant_topics:
        follow_up_templates.extend([
            "How do I apply this to a physical robot?",
            "What hardware considerations should I keep in mind?",
            "How does this concept relate to other robotics principles?",
            "What are the next steps to implement this in a real system?"
        ])
    else:
        # General follow-up questions if no specific topic is detected
        follow_up_templates.extend([
            "How can I apply this concept in practice?",
            "What are the next steps I should take?",
            "What are some common challenges with this topic?",
            "How does this relate to other concepts in robotics/AI?"
        ])

    # Select 2-3 random follow-up questions to avoid overwhelming the user
    import random
    selected_follow_ups = random.sample(follow_up_templates, min(3, len(follow_up_templates)))

    # Format the follow-up questions
    result = ""
    for i, question in enumerate(selected_follow_ups, 1):
        result += f"{i}. {question}\n"

    return result


def rag_chat(query: str, llm_choice: str = "claude", collection_name: str = "rag_embedding"):
    """Main RAG function that searches context and generates response."""
    logger.info(f"Searching in collection: {collection_name}")
    # Search for relevant chunks
    relevant_chunks = search_qdrant(query, collection_name)

    # Always define query_lower for use in both conditions
    query_lower = query.lower()

    # Check if the query is a conversational/presence check query
    conversational_queries = [
        "are you there", "are you there?", "you there", "there?", "there",
        "hello", "hi", "hey", "greetings", "are you available", "are you online",
        "can you hear me", "are you ready", "anyone there", "anyone there?",
        "is anyone there", "is anybody there", "anybody there"
    ]

    is_conversational_query = any(phrase in query_lower for phrase in conversational_queries)

    # Check if we have relevant chunks and if they meet a minimum similarity threshold
    # If the highest similarity score is very low, it means the query doesn't match the content well
    if relevant_chunks:
        # Get the highest similarity score
        max_score = max(chunk.get("score", 0) for chunk in relevant_chunks)

        # If it's a conversational query, return appropriate response regardless of content match
        if is_conversational_query:
            if "hello" in query_lower or "hi" in query_lower or "hey" in query_lower or "greetings" in query_lower:
                return "Hello! I'm your AI tutor for robotics and AI documentation. How can I help you learn today?"
            elif any(phrase in query_lower for phrase in ["are you there", "are you there?", "you there", "there?", "there", "are you available", "are you online", "can you hear me", "are you ready", "anyone there", "anyone there?", "is anyone there", "is anybody there", "anybody there"]):
                return "Yes, I'm here! I'm your AI tutor ready to help with questions about robotics and AI documentation."
            else:
                # For other conversational queries, return appropriate response
                return "Yes, I'm here! I'm an AI tutor specialized in helping you learn about robotics, AI, ROS 2, and related topics. How can I help you?"
        # If similarity score is too low, treat as no relevant content found
        elif max_score < 0.25:  # Increase threshold to be more strict about relevance
            if any(word in query_lower for word in ["hello", "hi", "hey", "greetings"]):
                return "Hi there! I'm your AI tutor for robotics and AI. It looks like I might not have the specific information you're looking for. I'd be happy to help with robotics, AI, or ROS 2 questions though!"
            elif any(phrase in query_lower for phrase in ["are you there", "are you there?", "you there", "there?", "there", "are you available", "are you online", "can you hear me", "are you ready", "anyone there", "anyone there?", "is anyone there", "is anybody there", "anybody there"]):
                return "Yes, I'm here! I'm your AI tutor ready to help with questions about robotics and AI documentation."
            elif any(phrase in query_lower for phrase in ["who are you", "what are you", "introduce yourself", "what do you do"]):
                return "I'm an AI tutor specialized in helping you learn about robotics, AI, ROS 2, and related topics. I can provide explanations from the indexed documentation and guide your learning journey."
            elif any(word in query_lower for word in ["bye", "goodbye", "exit", "quit"]):
                return "Goodbye! Feel free to ask more questions about robotics and AI whenever you need help."
            else:
                # For out-of-domain queries, provide a more helpful response
                if any(word in query_lower for word in ["weather", "today", "time", "date", "joke", "song", "game", "movie", "sport", "news"]):
                    return "I'm an AI tutor specialized in robotics and AI documentation. I can't provide information about general topics like weather, time, or entertainment. I can help you learn about robotics, AI, ROS 2, and related technical topics."
                else:
                    return "I'm sorry, I don't have information about that topic. I specialize in helping with robotics, AI, and ROS 2. Could you ask me something about those subjects instead?"
        else:
            # Continue with the original logic for relevant content
            pass
    else:
        # No chunks found
        if any(word in query_lower for word in ["hello", "hi", "hey", "greetings"]):
            return "Hello! I'm your AI tutor for robotics and AI documentation. Please make sure the knowledge base has been properly indexed with documentation content."
        elif any(phrase in query_lower for phrase in ["are you there", "are you there?", "you there", "there?", "there", "are you available", "are you online", "can you hear me", "are you ready", "anyone there", "anyone there?", "is anyone there", "is anybody there", "anybody there"]):
            return "Yes, I'm here! I'm your AI tutor ready to help with questions about robotics and AI documentation."
        elif any(phrase in query_lower for phrase in ["who are you", "what are you", "introduce yourself", "what do you do"]):
            return "I'm an AI tutor specialized in helping you learn about robotics, AI, ROS 2, and related topics. I can provide explanations from the indexed documentation and guide your learning journey."
        elif any(word in query_lower for word in ["bye", "goodbye", "exit", "quit"]):
            return "Goodbye! Feel free to ask more questions about robotics and AI whenever you need help."
        else:
            return "I'm sorry, I couldn't find relevant information about that in my knowledge base. I focus on robotics, AI, and ROS 2 topics. Is there something specific about robotics I can help you with instead?"

    # Process relevant chunks to extract context while preserving URLs and metadata
    # Shuffle the relevant chunks to create some variation
    shuffled_chunks = relevant_chunks.copy()
    random.shuffle(shuffled_chunks)

    # Take a random subset of chunks or rotate which chunks we use
    if len(shuffled_chunks) > 3:
        # Randomly select 3 chunks instead of all of them
        selected_chunks = random.sample(shuffled_chunks, 3)
    else:
        selected_chunks = shuffled_chunks

    # Extract context and relevant URLs
    context_parts = []
    urls_found = set()  # Use a set to avoid duplicate URLs
    for chunk in selected_chunks:
        text = chunk["text"]
        context_parts.append(text)

        # Extract URLs from the chunk if they exist
        import re
        url_matches = re.findall(r'Source URL: (https?://[^\s\n]+)', text)
        for url in url_matches:
            urls_found.add(url)

    # Combine the selected chunks as context
    context = "\n\n".join(context_parts)

    # Add variation by taking different lengths of context
    context_length_options = [500, 600, 700, 800]
    max_length = random.choice(context_length_options)
    context = context[:max_length]

    # Clean the context for LLM consumption while preserving essential info
    import re
    clean_context = re.sub(r'Source URL: https?://[^\n]+\n?', '', context)  # Remove source URLs from context sent to LLM
    clean_context = re.sub(r'Section: [^\n]+\n?', '', clean_context)  # Remove section lines
    clean_context = re.sub(r'On this page[^\n]*\n?', '', clean_context)  # Remove "On this page" references
    clean_context = re.sub(r'Module \d+:?[^\n]*\n?', '', clean_context)  # Remove module/chapter references
    clean_context = re.sub(r'Chapter \d+:?[^\n]*\n?', '', clean_context)
    clean_context = re.sub(r'^#+\s*[^\n]*\n?', '', clean_context, flags=re.MULTILINE)  # Remove headings
    clean_context = re.sub(r'\s+', ' ', clean_context)  # Normalize whitespace
    clean_context = clean_context.strip()

    # Generate response based on chosen LLM
    if llm_choice.lower() == "claude":
        response = generate_response_claude(clean_context, query)
        if "Error: Claude API key not configured" in response:
            # When API key is not configured, generate a tutor-style response based on the context
            logger.warning("Claude API not configured, generating tutor-style response")
            # Create a tutor-style response with documentation links

            # Add more conversational and friendly tutor-style responses
            response_templates = [
                f"Hey there! From what I know, {clean_context[:300]}... This is an important concept in robotics that can help you understand how things work.",
                f"Great question! Based on the documentation, {clean_context[:280]}... This is a fundamental part of how robotic systems operate.",
                f"I'm happy to help! {clean_context[:260]}... This is really useful knowledge for working with robots.",
                f"That's an interesting topic! {clean_context[:270]}... Understanding this will definitely help you in robotics.",
                f"Good question! {clean_context[:250]}... This concept is key to how many robotic systems function.",
                f"Let me explain that for you. {clean_context[:290]}... This is essential knowledge for robotics work.",
                f"I'd be glad to explain. {clean_context[:275]}... This is a core concept in robotics that you'll use often.",
                f"Sure thing! {clean_context[:265]}... This is really important for understanding robot systems."
            ]

            # Randomly select a response template to add variation
            base_response = random.choice(response_templates)
        else:
            base_response = response
    elif llm_choice.lower() == "gemini":
        response = generate_response_gemini(clean_context, query)
        if "Error: Gemini API key not configured" in response or "Error generating response" in response:
            # When API key is not configured, generate a tutor-style response based on the context
            logger.warning("Gemini API not configured, generating tutor-style response")
            # Create a tutor-style response with documentation links

            # Add more conversational and friendly tutor-style responses
            response_templates = [
                f"Hey there! From what I know, {clean_context[:300]}... This is an important concept in robotics that can help you understand how things work.",
                f"Great question! Based on the documentation, {clean_context[:280]}... This is a fundamental part of how robotic systems operate.",
                f"I'm happy to help! {clean_context[:260]}... This is really useful knowledge for working with robots.",
                f"That's an interesting topic! {clean_context[:270]}... Understanding this will definitely help you in robotics.",
                f"Good question! {clean_context[:250]}... This concept is key to how many robotic systems function.",
                f"Let me explain that for you. {clean_context[:290]}... This is essential knowledge for robotics work.",
                f"I'd be glad to explain. {clean_context[:275]}... This is a core concept in robotics that you'll use often.",
                f"Sure thing! {clean_context[:265]}... This is really important for understanding robot systems."
            ]

            # Randomly select a response template to add variation
            base_response = random.choice(response_templates)
        else:
            base_response = response
    else:
        # Default to Claude if invalid choice
        response = generate_response_claude(clean_context, query)
        if "Error: Claude API key not configured" in response:
            # When API key is not configured, generate a tutor-style response based on the context
            logger.warning("Claude API not configured, generating tutor-style response")
            # Create a tutor-style response with documentation links

            # Add more conversational and friendly tutor-style responses
            response_templates = [
                f"Hey there! From what I know, {clean_context[:300]}... This is an important concept in robotics that can help you understand how things work.",
                f"Great question! Based on the documentation, {clean_context[:280]}... This is a fundamental part of how robotic systems operate.",
                f"I'm happy to help! {clean_context[:260]}... This is really useful knowledge for working with robots.",
                f"That's an interesting topic! {clean_context[:270]}... Understanding this will definitely help you in robotics.",
                f"Good question! {clean_context[:250]}... This concept is key to how many robotic systems function.",
                f"Let me explain that for you. {clean_context[:290]}... This is essential knowledge for robotics work.",
                f"I'd be glad to explain. {clean_context[:275]}... This is a core concept in robotics that you'll use often.",
                f"Sure thing! {clean_context[:265]}... This is really important for understanding robot systems."
            ]

            # Randomly select a response template to add variation
            base_response = random.choice(response_templates)
        else:
            base_response = response

    # Add documentation links to the response if URLs were found
    if urls_found:
        # Create a nicely formatted list of documentation links
        links_text = "\n\n📚 For more detailed information, see the documentation:\n"
        for i, url in enumerate(urls_found, 1):
            links_text += f"{i}. {url}\n"

        # Append the links to the main response
        final_response = base_response + links_text
    else:
        final_response = base_response

    # Add follow-up questions to guide the learning journey
    follow_up_questions = generate_follow_up_questions(query, clean_context)
    if follow_up_questions:
        final_response += f"\n\n🤔 **SUGGESTED NEXT STEPS**:\n" + follow_up_questions

    return final_response


# Flask API Endpoints
@app.route('/chat', methods=['POST'])
def chat_endpoint():
    """API endpoint for RAG chat."""
    try:
        data = request.get_json()

        if not data or 'query' not in data:
            return jsonify({'error': 'Query is required'}), 400

        query = data.get('query')
        llm_choice = data.get('llm', 'claude')  # Default to Claude
        collection_name = data.get('collection', 'rag_embedding')

        response = rag_chat(query, llm_choice, collection_name)

        return jsonify({
            'query': query,
            'response': response,
            'llm_used': llm_choice
        })
    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        return jsonify({'error': f'Internal server error: {str(e)}'}), 500


@app.route('/index', methods=['POST'])
def index_endpoint():
    """API endpoint to trigger indexing of a Docusaurus site."""
    try:
        data = request.get_json()

        if not data or 'base_url' not in data:
            return jsonify({'error': 'base_url is required'}), 400

        base_url = data.get('base_url')
        collection_name = data.get('collection', 'rag_embedding')

        # Create collection if it doesn't exist
        create_collection(collection_name)

        # Fetch URLs
        logger.info(f"Fetching URLs from {base_url}...")
        urls = get_all_urls(base_url)

        if not urls:
            return jsonify({'error': 'No URLs found or an error occurred'}), 400

        # Process URLs with deterministic ordering
        total_chunks = 0
        processed_urls = 0
        failed_urls = 0
        current_doc_id = 0  # Global counter for unique chunk IDs across all documents

        for i, url in enumerate(urls):
            logger.info(f"Processing URL {i+1}/{len(urls)}: {url}")

            # Ingest content from URL with metadata
            ingestion_result = ingest_url_content(url)

            if ingestion_result['status'] == 'success' and ingestion_result['content']:
                logger.info(f"Successfully ingested content from {url} ({ingestion_result['word_count']} words)")

                # Add URL and section metadata to the text before chunking
                text_with_metadata = f"Source URL: {url}\nSection: {ingestion_result['section_id']}\n\n{ingestion_result['content']}"

                logger.info("Chunking text...")
                # Use global counter to ensure unique chunk IDs across all documents
                chunks = chunk_text(text_with_metadata, doc_start_id=current_doc_id)

                if chunks:
                    # Add URL and section metadata to each chunk
                    for chunk in chunks:
                        chunk['url'] = url
                        chunk['section'] = ingestion_result['section_id']

                    logger.info(f"Created {len(chunks)} chunks. Embedding them now...")
                    embedded_chunks = embed(chunks)

                    if embedded_chunks:
                        logger.info(f"Successfully embedded {len(embedded_chunks)} chunks.")
                        save_chunk_to_qdrant(embedded_chunks, collection_name)
                        total_chunks += len(embedded_chunks)
                        # Update the global counter for the next document
                        current_doc_id += len(embedded_chunks)
                        processed_urls += 1
                    else:
                        logger.warning(f"Embedding failed for URL: {url}")
                        failed_urls += 1
                else:
                    logger.warning(f"No chunks created for URL: {url}")
                    failed_urls += 1
            else:
                logger.warning(f"No content extracted from URL: {url}")
                failed_urls += 1

        return jsonify({
            'message': f'Indexing completed',
            'urls_processed': processed_urls,
            'urls_failed': failed_urls,
            'total_urls_found': len(urls),
            'total_chunks_indexed': total_chunks,
            'collection_name': collection_name,
            'status': 'completed' if failed_urls == 0 else 'partial_success'
        })
    except Exception as e:
        logger.error(f"Error in index endpoint: {str(e)}")
        return jsonify({'error': f'Internal server error: {str(e)}'}), 500


@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint."""
    return jsonify({
        'status': 'healthy',
        'message': 'RAG Chatbot API is running'
    })


@app.route('/collections', methods=['GET'])
def list_collections():
    """List all available Qdrant collections."""
    try:
        collections = qdrant.get_collections()
        collection_names = [coll.name for coll in collections.collections]
        return jsonify({
            'collections': collection_names,
            'count': len(collection_names)
        })
    except Exception as e:
        logger.error(f"Error listing collections: {str(e)}")
        return jsonify({
            'error': f'Error listing collections: {str(e)}'
        }), 500


@app.route('/collections/<collection_name>', methods=['GET'])
def collection_info(collection_name):
    """Get information about a specific collection."""
    try:
        collection_info = qdrant.get_collection(collection_name)
        # Return basic information about the collection - avoid attribute errors
        return jsonify({
            'name': collection_name,
            'status': 'available',
            'points_count': getattr(collection_info, 'points_count', 'unknown')
        })
    except Exception as e:
        logger.error(f"Error getting collection info: {str(e)}")
        return jsonify({
            'error': f'Collection {collection_name} not found: {str(e)}'
        }), 404


@app.route('/ingest', methods=['POST'])
def ingest_endpoint():
    """API endpoint to ingest content from specific URLs."""
    try:
        data = request.get_json()

        if not data or ('urls' not in data and 'base_url' not in data):
            return jsonify({'error': 'Either "urls" array or "base_url" is required'}), 400

        # Get URLs either from direct array or by fetching from base_url
        if 'urls' in data:
            urls = data.get('urls')
        else:
            base_url = data.get('base_url')
            urls = get_all_urls(base_url)
            if not urls:
                return jsonify({'error': 'No URLs found or an error occurred'}), 400

        # Ingest content from all URLs
        ingestion_results = ingest_multiple_urls(urls)

        # Filter successful results
        successful_results = [r for r in ingestion_results if r['status'] == 'success']
        failed_results = [r for r in ingestion_results if r['status'] == 'failed']

        return jsonify({
            'message': 'Ingestion completed',
            'total_urls': len(urls),
            'successful_ingestions': len(successful_results),
            'failed_ingestions': len(failed_results),
            'results': ingestion_results,
            'status': 'completed' if len(failed_results) == 0 else 'partial_success'
        })
    except Exception as e:
        logger.error(f"Error in ingest endpoint: {str(e)}")
        return jsonify({'error': f'Internal server error: {str(e)}'}), 500

def main():
    """Main function to execute the indexing pipeline."""
    base_url = "https://physiacal-ai-and-humanoid-book-hack.vercel.app"
    print(f"Fetching URLs from {base_url}...")
    urls = get_all_urls(base_url)
    if urls:
        print(f"Found {len(urls)} URLs. Processing the first one for embedding...")

        # Process only the first URL for this test
        first_url = urls[0]
        print(f"\n--- Processing URL: {first_url} ---")
        text = extract_text_from_url(first_url)

        if text:
            print("--- Chunking Text ---")
            chunks = chunk_text(text, doc_start_id=0)  # Use 0 as start ID for single document

            if chunks:
                print(f"Created {len(chunks)} chunks. Embedding them now...")
                embedded_chunks = embed(chunks)
                if embedded_chunks:
                    print(f"Successfully embedded {len(embedded_chunks)} chunks.")
                    print("Example embedding vector (first 5 dimensions):")
                    print(embedded_chunks[0]["embedding"][:5])
                else:
                    print("Embedding failed.")
            else:
                print("No chunks created.")
        else:
            print("No text extracted.")
    else:
        print("No URLs found or an error occurred.")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "serve":
        # Run the Flask app
        app.run(host='0.0.0.0', port=8000, debug=True)
        print("RAG Chatbot API is running on http://localhost:8000")
        print("Available endpoints:")
        print("  POST /chat - Chat with the RAG system")
        print("  POST /index - Index a Docusaurus site")
        print("  POST /ingest - Ingest content from specific URLs")
        print("  GET /health - Health check")
        print("  GET /collections - List available collections")
        print("  GET /collections/<name> - Get collection info")
    else:
        # Run the original main function
        main()