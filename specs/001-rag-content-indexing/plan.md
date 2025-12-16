# Implementation Plan: Content Indexing for RAG

**Branch**: `001-rag-content-indexing` | **Date**: 2025-12-11 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/001-rag-content-indexing/spec.md`

## Summary

The technical plan is to create a Python script that fetches all published Docusaurus book URLs from a given site, extracts clean text content, chunks the text, generates Cohere embeddings, and stores the vectors and metadata in a Qdrant collection for Retrieval Augmented Generation (RAG). The entire pipeline will be contained within a single `main.py` file inside a `backend` folder and will be managed using `uv` for package management.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: 
- `uv` (package manager)
- `requests` (for fetching URLs)
- `beautifulsoup4` (for HTML parsing and text extraction)
- `tiktoken` (for token-based text chunking)
- `cohere` (for embedding generation)
- `qdrant-client` (for vector storage)
**Storage**: Qdrant Cloud or local instance
**Testing**: `pytest` for smoke tests
**Target Platform**: Any system with Python 3.11
**Project Type**: Single script in a `backend` folder
**Performance Goals**: Generate Cohere embeddings in batches of 64-128 with retry/backoff.
**Constraints**: 
- All logic in a single `main.py` file.
- All secrets and configurations managed via environment variables.
**Scale/Scope**: Initial full indexing of a single Docusaurus site, with a smoke test on 5 pages.

## Constitution Check

*Gates checked and passed. The plan adheres to the project's constitution.*

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-content-indexing/
├── plan.md              # This file
├── research.md          # To be created if research is needed
├── data-model.md        # To be created for data schema details
├── quickstart.md        # To be created with setup and run instructions
├── contracts/           # To be created for any API contracts
└── tasks.md             # To be created by /sp.tasks
```

### Source Code (repository root)

```text
backend/
└── main.py

tests/
├── smoke_test.py
└── .env.example
```

**Structure Decision**: The structure is a single `backend` folder containing the `main.py` script, as requested by the user. A `tests` folder will contain a smoke test and an example environment file.

## Function Signatures (as per user request)

```python
def get_all_urls(base_url: str) -> list[str]:
    """Fetches all book URLs from the Docusaurus site."""
    pass

def extract_text_from_url(url: str) -> str:
    """Extracts clean text from a given URL."""
    pass

def chunk_text(text: str, chunk_size: int = 700, overlap: int = 120) -> list[dict]:
    """Chunks text into smaller pieces with deterministic IDs."""
    pass

def embed(chunks: list[dict], batch_size: int = 64) -> list[dict]:
    """Generates Cohere embeddings for text chunks in batches."""
    pass

def create_collection(collection_name: str = "rag_embedding"):
    """Creates a Qdrant collection with the correct vector size and metadata fields."""
    pass

def save_chunk_to_qdrant(chunks: list[dict], collection_name: str = "rag_embedding"):
    """Upserts a batch of chunks with their embeddings to Qdrant."""
    pass

def main():
    """Main function to execute the indexing pipeline."""
    pass
```

## Risk Analysis

| Risk | Mitigation |
|------|------------|
| Cohere/Qdrant API failures | Implement retry logic with exponential backoff for transient errors. Log persistent errors. |
| Inconsistent HTML structure on Docusaurus site | Make the text extraction logic in `extract_text_from_url` robust to handle variations in HTML structure. Add logging for pages with unexpected structures. |
| Performance issues with large-scale indexing | Use batching for embedding generation and Qdrant upserts. Optimize text extraction. |
| Secrets management | Use environment variables and a `.env.example` file to manage API keys and other secrets. |
| URL discovery might be incomplete | The `get_all_urls` function should be designed to crawl the site map or navigate through all linked pages to discover all book URLs. |

## Evaluation and Validation

A smoke test will be implemented in `tests/smoke_test.py`. This test will:
1.  Index 5 pages from the provided Docusaurus site.
2.  Perform a sample semantic search query against the indexed data.
3.  Verify that the search results contain relevant chunks from the indexed pages.

This will provide a quick validation of the end-to-end pipeline.
