import React from 'react';
import SignupForm from '../../components/Auth/SignupForm';
import '../../css/auth.css';

const SignupPage = () => {
  const handleSignupSuccess = () => {
    // Redirect to a success page or dashboard
    window.location.href = '/';
  };

  return (
    <div className="auth-page">
      <div className="container">
        <SignupForm onSignupSuccess={handleSignupSuccess} />
        <div className="auth-link-section">
          <p>Already have an account? <a href="/auth/signin">Sign in here</a></p>
        </div>
      </div>
    </div>
  );
};

export default SignupPage;