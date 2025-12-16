# RAG Chatbot API Usage Examples

## Starting the Server

```bash
cd backend
python main.py serve
```

The API will be available at `http://localhost:5000`

## API Usage Examples

### 1. Health Check
```bash
curl -X GET http://localhost:5000/health
```

### 2. Index a Docusaurus Site
```bash
curl -X POST http://localhost:5000/index \
  -H "Content-Type: application/json" \
  -d '{
    "base_url": "https://your-docusaurus-site.com",
    "collection": "rag_embedding"
  }'
```

### 3. Chat with Claude
```bash
curl -X POST http://localhost:5000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is the main topic of the documentation?",
    "llm": "claude",
    "collection": "rag_embedding"
  }'
```

### 4. Chat with Gemini
```bash
curl -X POST http://localhost:5000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain the key concepts in the documentation",
    "llm": "gemini",
    "collection": "rag_embedding"
  }'
```

## Python Client Example

```python
import requests

# Index a site
response = requests.post('http://localhost:5000/index', json={
    'base_url': 'https://your-docusaurus-site.com',
    'collection': 'rag_embedding'
})
print(response.json())

# Chat with the RAG system
response = requests.post('http://localhost:5000/chat', json={
    'query': 'Your question here',
    'llm': 'claude',  # or 'gemini'
    'collection': 'rag_embedding'
})
print(response.json())
```

## Environment Variables

Create a `.env` file in the `backend` directory:

```env
COHERE_API_KEY="your_cohere_api_key"
QDRANT_API_KEY="your_qdrant_api_key"
QDRANT_URL="your_qdrant_url"
ANTHROPIC_API_KEY="your_anthropic_api_key"  # Optional
GEMINI_API_KEY="your_gemini_api_key"        # Optional
```