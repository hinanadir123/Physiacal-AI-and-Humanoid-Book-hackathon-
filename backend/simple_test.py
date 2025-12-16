#!/usr/bin/env python3
"""
Simple test to verify varied responses without Unicode encoding issues
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

def simple_test():
    """Simple test to verify varied responses"""
    print("Testing varied responses for the same query...")

    query = "what is ai"
    responses = []

    print(f"Testing query: '{query}' (6 times)")
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
        print("SUCCESS: Same query produces varied response formats!")
    else:
        print("ISSUE: Same query produces identical formats")

    print("\nTesting different query types...")
    test_queries = ["what is robotics", "how to create a robot", "explain ai concepts"]

    for query in test_queries:
        try:
            response = rag_chat(query, collection_name="rag_embedding")
            first_line = response.split('\n')[0]
            print(f"Query '{query}': {first_line[:70]}{'...' if len(first_line) > 70 else ''}")
        except Exception as e:
            print(f"Query '{query}': Error - {str(e)}")

if __name__ == "__main__":
    simple_test()