#!/usr/bin/env python3
"""
Test script to verify the chatbot functionality and API responses
"""
import sys
import os

# Add the current directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

import main
from main import rag_chat, search_qdrant

def test_chat_responses():
    """Test different types of queries to verify varied responses"""
    print("Testing chatbot responses...")

    # Test queries that should return different responses when no knowledge base is available
    test_queries = [
        "Hello",
        "Hi there",
        "What is robotics?",
        "How to create a robot?",
        "Thanks for help",
        "Can you show me an example?",
        "Explain AI concepts"
    ]

    print("\nTesting queries with no knowledge base (should show varied responses):")
    print("="*60)

    for i, query in enumerate(test_queries, 1):
        print(f"\n{i}. Query: '{query}'")
        try:
            response = rag_chat(query, collection_name="rag_embedding")
            print(f"   Response: {response[:100]}..." if len(response) > 100 else f"   Response: {response}")
        except Exception as e:
            print(f"   Error: {str(e)}")

    print("\n" + "="*60)
    print("Testing completed. If API keys are configured and knowledge base is indexed,")
    print("responses should be based on the retrieved context and vary by query.")
    print("\nTo properly use the chatbot:")
    print("1. Make sure ANTHROPIC_API_KEY or GEMINI_API_KEY are set in .env")
    print("2. Index your documentation using the /index endpoint")
    print("3. Test with various queries to see contextually relevant responses")

if __name__ == "__main__":
    test_chat_responses()