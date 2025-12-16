# Implementation Tasks: Content Indexing for RAG

**Feature**: Content Indexing for RAG | **Branch**: `001-rag-content-indexing` | **Date**: 2025-12-13 | **Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Dependencies

- User Story 1 (P1) must be completed before User Story 2 (P2)
- URL fetching functionality required before text extraction
- Text extraction required before chunking
- Chunking required before embedding
- Embedding required before Qdrant storage

## Parallel Execution Examples

- **US1 - Chunking and Embedding**: `chunk_text()` and `embed()` functions can be developed in parallel with `extract_text_from_url()`
- **US1 - Storage**: Qdrant collection creation can be developed in parallel with embedding
- **US2 - Testing**: Test scenarios can be developed in parallel with implementation

## Implementation Strategy

**MVP Scope**: User Story 1 (Core indexing functionality) with basic chunking, embedding, and storage. This provides the essential RAG capability with minimal viable functionality.

**Incremental Delivery**:
1. MVP: Basic URL fetching → text extraction → chunking → storage
2. Enhancement: Add embedding generation
3. Polish: Add error handling, logging, and quality verification

---

## Phase 1: Setup

- [ ] T001 Create backend directory structure
- [ ] T002 Create requirements.txt with dependencies (requests, beautifulsoup4, tiktoken, cohere, qdrant-client, flask, anthropic, google-generativeai)
- [ ] T003 Create .env.example with required environment variables
- [ ] T004 Create initial backend/main.py file with imports and configuration

## Phase 2: Foundational

- [ ] T005 Create URL fetching function `get_all_urls(base_url)` in backend/main.py
- [ ] T006 Create text extraction function `extract_text_from_url(url)` in backend/main.py with HTML cleaning
- [ ] T007 Create chunking function `chunk_text(text, chunk_size=700, overlap=120)` in backend/main.py
- [ ] T008 Create embedding function `embed(chunks, batch_size=64)` in backend/main.py
- [ ] T009 Create Qdrant collection function `create_collection(collection_name)` in backend/main.py
- [ ] T010 Create Qdrant storage function `save_chunk_to_qdrant(chunks, collection_name)` in backend/main.py

## Phase 3: [US1] Index Docusaurus Content for Retrieval

**Goal**: Implement the core indexing pipeline that fetches Docusaurus content, chunks it, generates embeddings, and stores in Qdrant.

**Independent Test**: The indexing script can be run against a Docusaurus site, and the Qdrant database can be queried to confirm that content chunks and their embeddings are present and correctly associated with metadata.

- [ ] T011 [P] [US1] Implement deterministic chunking with 700 token size and 120 token overlap in chunk_text function
- [ ] T012 [P] [US1] Assign unique identifiers to each chunk using position-based IDs in chunk_text function
- [ ] T013 [P] [US1] Attach metadata (URL, section) to each chunk in the indexing pipeline
- [ ] T014 [US1] Create main indexing function that orchestrates URL fetching → text extraction → chunking → embedding → storage
- [ ] T015 [US1] Add proper error handling for unreachable URLs in get_all_urls function
- [ ] T016 [US1] Handle small documents that may not meet chunking requirements
- [ ] T017 [US1] Implement retry logic for Cohere API rate limits in embed function
- [ ] T018 [US1] Add logging for indexing process with sample logs for 5 pages
- [ ] T019 [US1] Create API endpoint POST /index for triggering indexing of a Docusaurus site
- [ ] T020 [US1] Validate that chunks are created with token counts between 650-750 tokens and overlap of 110-130 tokens

## Phase 4: [US2] Verify Indexing Quality

**Goal**: Implement functionality to verify the quality and correctness of the indexed content, chunking, and embeddings.

**Independent Test**: A simple end-to-end test query can be performed against the Qdrant instance, and the returned chunks can be manually inspected for relevance and chunking logic.

- [ ] T021 [P] [US2] Create search function `search_qdrant(query, collection_name, top_k)` in backend/main.py
- [ ] T022 [P] [US2] Create RAG chat function `rag_chat(query, llm_choice, collection_name)` in backend/main.py
- [ ] T023 [US2] Create API endpoint POST /chat for RAG queries
- [ ] T024 [US2] Create API endpoint GET /health for system status
- [ ] T025 [US2] Add quality verification for end-to-end test queries
- [ ] T026 [US2] Implement meaningful chunk verification (ensure context preservation)
- [ ] T027 [US2] Create documentation for pipeline reproducibility
- [ ] T028 [US2] Add Claude and Gemini integration for response generation
- [ ] T029 [US2] Create test endpoint for verifying indexing quality

## Phase 5: Polish & Cross-Cutting Concerns

- [ ] T030 Add comprehensive error handling throughout the application
- [ ] T031 Add input validation for API endpoints
- [ ] T032 Create README.md with setup and usage instructions
- [ ] T033 Add configuration validation for environment variables
- [ ] T034 Create API usage examples documentation
- [ ] T035 Add proper logging configuration with log levels
- [ ] T036 Create smoke tests for the indexing pipeline in tests/smoke_test.py
- [ ] T037 Add graceful degradation when Qdrant is unavailable
- [ ] T038 Create documentation for deployment and maintenance
- [ ] T039 Add integration with Docusaurus website as a chatbot component