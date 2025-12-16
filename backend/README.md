# RAG Chatbot Backend

This backend implements a Retrieval Augmented Generation (RAG) chatbot that can index Docusaurus documentation sites and answer questions about their content using Claude and Gemini AI models.

## Features

- Index Docusaurus documentation sites by fetching URLs from sitemap.xml
- Extract clean text from web pages
- Chunk text with configurable size and overlap
- Generate embeddings using Cohere
- Store embeddings in Qdrant vector database
- Retrieve relevant content using semantic search
- Generate responses using Claude and/or Gemini AI models
- RESTful API endpoints for integration

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up environment variables in a `.env` file:
   ```env
   COHERE_API_KEY="your_cohere_api_key"
   QDRANT_API_KEY="your_qdrant_api_key"
   QDRANT_URL="your_qdrant_url"
   ANTHROPIC_API_KEY="your_anthropic_api_key"  # Optional
   GEMINI_API_KEY="your_gemini_api_key"        # Optional
   ```

## Usage

### Running the API Server

```bash
cd backend
python main.py serve
```

The API will be available at `http://localhost:5000`

### API Endpoints

#### Health Check
- `GET /health` - Check if the service is running

#### Indexing
- `POST /index` - Index a Docusaurus site

Request body:
```json
{
  "base_url": "https://your-docusaurus-site.com",
  "collection": "rag_embedding"
}
```

#### Chat
- `POST /chat` - Chat with the RAG system

Request body:
```json
{
  "query": "Your question here",
  "llm": "claude",  // or "gemini"
  "collection": "rag_embedding"
}
```

Response:
```json
{
  "query": "Your question here",
  "response": "AI-generated response based on indexed content",
  "llm_used": "claude"
}
```

## Running the Indexing Pipeline

To run just the indexing pipeline without starting the server:

```bash
python main.py
```

This will run the original indexing pipeline on the default site.

## Architecture

1. **Indexing Pipeline**:
   - Fetches URLs from sitemap.xml
   - Extracts clean text from each page
   - Chunks text using tiktoken
   - Generates embeddings with Cohere
   - Stores in Qdrant vector database

2. **Retrieval Pipeline**:
   - Receives user query
   - Generates embedding for query
   - Searches Qdrant for relevant chunks
   - Combines top results as context

3. **Generation Pipeline**:
   - Formats context and query for LLM
   - Sends to Claude or Gemini
   - Returns AI-generated response

## Supported LLMs

- **Claude**: Requires ANTHROPIC_API_KEY
- **Gemini**: Requires GEMINI_API_KEY

If both are configured, you can choose which one to use via the API. If only one is configured, it will be used by default.