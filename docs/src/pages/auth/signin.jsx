import React from 'react';
import SigninForm from '../../components/Auth/SigninForm';
import '../../css/auth.css';

const SigninPage = () => {
  const handleSigninSuccess = () => {
    // Redirect to dashboard or previous page
    window.location.href = '/';
  };

  return (
    <div className="auth-page">
      <div className="container">
        <SigninForm onSigninSuccess={handleSigninSuccess} />
        <div className="auth-link-section">
          <p>Don't have an account? <a href="/auth/signup">Sign up here</a></p>
        </div>
      </div>
    </div>
  );
};

export default SigninPage;