#!/usr/bin/env python3
"""
Comprehensive test to verify varied responses for different query types
"""
import sys
import os

# Add the current directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

import main
from main import rag_chat
import time

def test_different_query_types():
    """Test different types of queries to show varied responses"""
    print("Testing varied responses for different query types...")

    test_queries = [
        "hi",
        "hello",
        "what is robotics",
        "how to create a robot",
        "explain ai concepts",
        "show me an example",
        "what is ros2",
        "thanks"
    ]

    print(f"\nTesting {len(test_queries)} different query types:")
    print("="*70)

    for i, query in enumerate(test_queries, 1):
        print(f"\n{i:2}. Query: '{query}'")
        try:
            response = rag_chat(query, collection_name="rag_embedding")
            # Show just the beginning of response to identify the format
            first_line = response.split('\n')[0]
            print(f"    Format: {first_line[:60]}{'...' if len(first_line) > 60 else ''}")
            print(f"    Content preview: {response[:100]}{'...' if len(response) > 100 else ''}")
        except Exception as e:
            print(f"    Error: {str(e)}")
        time.sleep(0.1)  # Small delay

    print("\n" + "="*70)
    print("Test completed. Notice how response formats vary based on query type and randomization.")

def test_same_query_varied_responses():
    """Test that same query produces varied response formats"""
    print("\n\nTesting varied formats for the same query...")
    print("="*50)

    query = "what is ai"
    responses = []

    print(f"Query: '{query}' (tested 6 times)")
    print("-" * 50)

    for i in range(6):
        try:
            response = rag_chat(query, collection_name="rag_embedding")
            first_line = response.split('\n')[0]
            responses.append(first_line)
            print(f"{i+1}. {first_line[:70]}{'...' if len(first_line) > 70 else ''}")
        except Exception as e:
            print(f"{i+1}. Error: {str(e)}")
        time.sleep(0.1)

    unique_formats = set(responses)
    print(f"\nUnique response formats: {len(unique_formats)}/{len(responses)}")

    if len(unique_formats) > 1:
        print("✅ SUCCESS: Same query produces varied response formats!")
    else:
        print("❌ ISSUE: Same query produces identical formats")

if __name__ == "__main__":
    test_different_query_types()
    test_same_query_varied_responses()